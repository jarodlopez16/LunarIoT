"""
Lunar IoT Rover Pathfinding + Battery Planner (Python)

ROLE OF THIS FILE
-----------------
- Input:  A list of bounding boxes from vision (label, x, y, w, h) in pixels
- Output: A JSON mission plan with waypoints in Unreal coordinates

"""

import threading
import paho.mqtt.client as mqtt
import math
import json
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
import heapq

# =======================================
# GLOBAL CONFIGURATION
# =======================================

# 1. Camera image resolution in pixels.
#    Replace these with the actual resolution.
IMAGE_WIDTH_PX = 1280
IMAGE_HEIGHT_PX = 720

# 2. Physical map size in meters.
#    These MUST match the field / grid.
MAP_WIDTH_M = 8.0     # width of the playable area (meters)
MAP_HEIGHT_M = 6.0    # height of the playable area (meters)

# 3. Grid resolution in meters per cell.
#    This should match grid spacing.
CELL_SIZE_M = 0.5

# Derived grid size (number of cells).
GRID_WIDTH = int(round(MAP_WIDTH_M / CELL_SIZE_M))
GRID_HEIGHT = int(round(MAP_HEIGHT_M / CELL_SIZE_M))

assert abs(GRID_WIDTH * CELL_SIZE_M - MAP_WIDTH_M) < 1e-6
assert abs(GRID_HEIGHT * CELL_SIZE_M - MAP_HEIGHT_M) < 1e-6

# 4. Unreal scale.
#    If "1 Unreal unit = 1 cm", then 1 m = 100 units.
UNREAL_UNITS_PER_METER = 100.0

# 5. Battery model parameters.
BATTERY_VOLTAGE_V = 7.4
BATTERY_CAPACITY_AH = 5.0
BATTERY_CAPACITY_WH = BATTERY_VOLTAGE_V * BATTERY_CAPACITY_AH
BATTERY_CAPACITY_J = BATTERY_CAPACITY_WH * 3600.0

# 6. Safety SOC reserve (percent).
MIN_SOC_RESERVE = 10.0


# =======================================
# BASIC DATA STRUCTURES
# =======================================

@dataclass
class BBox:
    """
    Bounding box from vision (pixel coordinates).

    label: "rover", "blue", "red", "yellow", etc.
    x, y: top left corner in pixels
    w, h: width and height in pixels
    """
    label: str
    x: float
    y: float
    w: float
    h: float


@dataclass
class GridCell:
    ix: int
    iy: int


class GridMap:
    """
    Grid map with base movement costs and optional battery cells.
    cost[y][x] is the base movement cost for that cell.
    None means obstacle (not walkable).
    """

    def __init__(self, width: int, height: int,
                 cost: Optional[List[List[Optional[float]]]] = None) -> None:
        self.width = width
        self.height = height

        if cost is None:
            # DEFAULT: flat cost 1.0 everywhere, no obstacles.
            # TODO: Replace with your real cost grid from the chalkboard.
            self.cost = [[1.0 for _ in range(width)] for _ in range(height)]
        else:
            self.cost = cost

        self.battery_cells = set()  # set of (x, y) tuples for charging stations

    def in_bounds(self, cell: GridCell) -> bool:
        return 0 <= cell.ix < self.width and 0 <= cell.iy < self.height

    def is_walkable(self, cell: GridCell) -> bool:
        if not self.in_bounds(cell):
            return False
        return self.cost[cell.iy][cell.ix] is not None

    def movement_cost(self, from_cell: GridCell, to_cell: GridCell) -> float:
        """
        Base movement cost for going into 'to_cell'.
        Typically this comes from terrain cost (chalkboard numbers).
        """
        base = self.cost[to_cell.iy][to_cell.ix]
        if base is None:
            return math.inf
        return float(base)

    def add_battery_cell(self, ix: int, iy: int) -> None:
        """
        Mark a cell as a battery/charging station.
        """
        self.battery_cells.add((ix, iy))

    def is_battery(self, cell: GridCell) -> bool:
        return (cell.ix, cell.iy) in self.battery_cells

    def cell_center_world_m(self, cell: GridCell) -> Tuple[float, float]:
        """
        Convert a grid cell index to local map coordinates (meters).
        """
        x_m = (cell.ix + 0.5) * CELL_SIZE_M
        y_m = (cell.iy + 0.5) * CELL_SIZE_M
        return x_m, y_m


# =======================================
# PIXEL → LOCAL M → GRID (VISION → MAP)
# =======================================

# Homography matrix H (3x3) that maps pixel coordinates (px, py, 1)
# to local map coordinates (x_m, y_m, w).
#
# TODO: If the camera is calibrated, put the TRUE homography here.
# For now this is a pure scale mapping (top-down, no rotation).
H = [
    [MAP_WIDTH_M / IMAGE_WIDTH_PX, 0.0, 0.0],
    [0.0, MAP_HEIGHT_M / IMAGE_HEIGHT_PX, 0.0],
    [0.0, 0.0, 1.0],
]

def soc_drop_for_distance(battery_specs, distance_m, power_w, speed_mps=0.2):
    """
    Estimate SOC drop for a battery station using PyBaMM.

    battery_specs: dict with capacity_mAh, nominal_voltage
    distance_m: meters
    power_w: watts consumed by rover
    speed_mps: meters per second
    """
    capacity_ah = battery_specs["capacity_mAh"] / 1000.0
    nominal_voltage = battery_specs["nominal_voltage"]
    
    # Energy required to travel the distance
    time_sec = distance_m / speed_mps
    energy_used_j = power_w * time_sec
    energy_used_wh = energy_used_j / 3600.0
    
    # SOC drop in percent
    battery_energy_wh = capacity_ah * nominal_voltage
    soc_drop = 100.0 * (energy_used_wh / battery_energy_wh)
    
    return soc_drop

def add_obstacle_bbox(self, bbox: BBox):
    """
    Mark all cells covered by this bounding box as non-walkable.
    """
    # Convert top-left and bottom-right of bbox to local meters
    x1_m, y1_m = apply_homography(bbox.x, bbox.y)
    x2_m, y2_m = apply_homography(bbox.x + bbox.w, bbox.y + bbox.h)

    # Convert to grid indices
    ix1 = max(0, min(self.width - 1, int(x1_m / CELL_SIZE_M)))
    iy1 = max(0, min(self.height - 1, int(y1_m / CELL_SIZE_M)))
    ix2 = max(0, min(self.width - 1, int(x2_m / CELL_SIZE_M)))
    iy2 = max(0, min(self.height - 1, int(y2_m / CELL_SIZE_M)))

    for iy in range(iy1, iy2 + 1):
        for ix in range(ix1, ix2 + 1):
            self.cost[iy][ix] = None  # mark as obstacle

def apply_homography(px: float, py: float) -> Tuple[float, float]:
    """
    Apply homography H to pixel coordinates and return local map coordinates (meters).
    """
    x = H[0][0] * px + H[0][1] * py + H[0][2]
    y = H[1][0] * px + H[1][1] * py + H[1][2]
    w = H[2][0] * px + H[2][1] * py + H[2][2]
    if w == 0:
        raise ValueError("Homography produced w = 0")
    return x / w, y / w


def bbox_bottom_center(b: BBox) -> Tuple[float, float]:
    """
    Bottom-center of the box in pixel coordinates.
    This approximates the point where the object touches the ground.
    """
    cx = b.x + b.w / 2.0
    cy = b.y + b.h
    return cx, cy


def pixel_bbox_to_grid_cell(b: BBox, grid: GridMap) -> GridCell:
    """
    Vision → pixel bbox → local meters → grid cell.
    This is the pipeline from Jarod's detections into A* nodes.
    """
    px, py = bbox_bottom_center(b)
    x_m, y_m = apply_homography(px, py)

    ix = int(x_m / CELL_SIZE_M)
    iy = int(y_m / CELL_SIZE_M)

    ix = max(0, min(grid.width - 1, ix))
    iy = max(0, min(grid.height - 1, iy))
    return GridCell(ix, iy)


# =======================================
# BATTERY MODEL ABSTRACTION (PyBaMM HOOK)
# =======================================

class BatteryModel:
    """
    Abstract energy model for the rover.
    This is where PyBaMM connects.

    The planner calls:
        soc_drop = battery_model.soc_drop_for_distance(distance_m, soc_start)

    TODO: Replace the simple model with PyBaMM-based logic once ready.
    """

    def __init__(self, power_w=15.0, nominal_speed_mps=0.2, station_specs=None):
        self.power_w = power_w
        self.speed_mps = nominal_speed_mps
        self.station_specs = station_specs 

    def soc_drop_for_distance(self, distance_m, soc_start, station_label=None):
        if station_label and station_label in self.station_specs:
            specs = self.station_specs[station_label]
            return soc_drop_for_distance(specs, distance_m, self.power_w, self.speed_mps)
        # default model
        time_sec = distance_m / self.speed_mps
        energy_used_j = self.power_w * time_sec
        soc_drop = 100.0 * (energy_used_j / BATTERY_CAPACITY_J)
        return soc_drop


# =======================================
# A* WITH SOC CONSTRAINTS
# =======================================

def heuristic(a: GridCell, b: GridCell) -> float:
    dx = a.ix - b.ix
    dy = a.iy - b.iy
    return math.hypot(dx, dy)


def reconstruct_path(came_from: Dict[Tuple[int, int], Tuple[int, int]],
                     current: GridCell) -> List[GridCell]:
    path = [current]
    key = (current.ix, current.iy)
    while key in came_from:
        prev = came_from[key]
        current = GridCell(prev[0], prev[1])
        path.append(current)
        key = (current.ix, current.iy)
    path.reverse()
    return path


def astar_with_battery(
    grid: GridMap,
    start: GridCell,
    goal: GridCell,
    start_soc: float,
    battery_model: BatteryModel,
    min_soc: float = MIN_SOC_RESERVE,
) -> Tuple[List[GridCell], float, float]:
    """
    A* pathfinding on the grid with SOC constraints.

    Returns:
        path: list of GridCell from start to goal.
        total_cost: sum of movement costs.
        final_soc: SOC at the goal.

    If no feasible path exists (with SOC >= min_soc), returns ([], inf, start_soc).
    """
    start_key = (start.ix, start.iy)
    goal_key = (goal.ix, goal.iy)

    open_heap: List[Tuple[float, Tuple[int, int]]] = []
    heapq.heappush(open_heap, (0.0, start_key))

    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
    g_score: Dict[Tuple[int, int], float] = {start_key: 0.0}
    soc_at: Dict[Tuple[int, int], float] = {start_key: start_soc}
    closed = set()

    while open_heap:
        _, current_key = heapq.heappop(open_heap)
        if current_key in closed:
            continue
        closed.add(current_key)

        current_cell = GridCell(current_key[0], current_key[1])

        if current_key == goal_key:
            path = reconstruct_path(came_from, current_cell)
            return path, g_score[current_key], soc_at[current_key]

        neighbors = [
            GridCell(current_cell.ix + 1, current_cell.iy),
            GridCell(current_cell.ix - 1, current_cell.iy),
            GridCell(current_cell.ix, current_cell.iy + 1),
            GridCell(current_cell.ix, current_cell.iy - 1),
        ]

        for nb in neighbors:
            nb_key = (nb.ix, nb.iy)
            if not grid.is_walkable(nb):
                continue

            # distance in meters between center of cells
            dx_cells = nb.ix - current_cell.ix
            dy_cells = nb.iy - current_cell.iy
            dist_m = math.hypot(dx_cells, dy_cells) * CELL_SIZE_M

            current_soc = soc_at[current_key]
            soc_drop = battery_model.soc_drop_for_distance(dist_m, current_soc)
            nb_soc = current_soc - soc_drop

            if nb_soc < min_soc:
                # this step would violate SOC reserve, skip
                continue

            move_cost = grid.movement_cost(current_cell, nb)
            tentative_g = g_score[current_key] + move_cost

            if nb_key not in g_score or tentative_g < g_score[nb_key]:
                g_score[nb_key] = tentative_g
                # simple assumption: if we enter a battery cell, we fully charge
                soc_at[nb_key] = 100.0 if grid.is_battery(nb) else nb_soc
                came_from[nb_key] = current_key
                f_score = tentative_g + heuristic(nb, goal)
                heapq.heappush(open_heap, (f_score, nb_key))

    # no path found
    return [], math.inf, start_soc


# =======================================
# MISSION PLANNING OVER MULTIPLE OBJECTIVES
# =======================================

def select_bbox_by_label(bboxes: List[BBox], label: str) -> Optional[BBox]:
    for b in bboxes:
        if b.label.lower() == label.lower():
            return b
    return None


def plan_mission_ordered(
    grid: GridMap,
    bboxes: List[BBox],
    mission_objective_labels: List[str],
    rover_label: str,
    start_soc: float,
    battery_model: BatteryModel,
) -> Tuple[List[GridCell], float, float]:
    """
    Plan a mission that visits objectives in the given order.

    - rover_label: label of the rover in the vision data (e.g., "rover")
    - mission_objective_labels: labels of objectives (e.g., ["blue", "red", "yellow"])

    Returns:
        full_path: concatenated path over all legs
        total_cost: sum of all leg costs
        final_soc: SOC after the last objective
    """
    bbox_rover = select_bbox_by_label(bboxes, rover_label)
    if bbox_rover is None:
        raise ValueError("Rover bounding box not found in vision data")

    current_cell = pixel_bbox_to_grid_cell(bbox_rover, grid)
    soc = start_soc
    full_path: List[GridCell] = []
    total_cost = 0.0

    for label in mission_objective_labels:
        bbox_goal = select_bbox_by_label(bboxes, label)
        if bbox_goal is None:
            raise ValueError(f"Objective bounding box for '{label}' not found")

        goal_cell = pixel_bbox_to_grid_cell(bbox_goal, grid)

        path_segment, cost_segment, soc = astar_with_battery(
            grid=grid,
            start=current_cell,
            goal=goal_cell,
            start_soc=soc,
            battery_model=battery_model,
            min_soc=MIN_SOC_RESERVE,
        )

        if not path_segment:
            raise RuntimeError(
                f"No feasible path to objective '{label}' with SOC >= {MIN_SOC_RESERVE}%"
            )

        # avoid duplicating the starting cell when concatenating
        if full_path:
            full_path.extend(path_segment[1:])
        else:
            full_path.extend(path_segment)

        total_cost += cost_segment
        current_cell = goal_cell

    return full_path, total_cost, soc


# =======================================
# EXPORT FOR UNREAL (WAYPOINT JSON)
# =======================================

def get_path_json(
    grid: GridMap,
    path: List[GridCell],
) -> str:
    """
    Convert a grid path into Unreal world coordinates and write mission_plan.json.

    This is the ONLY thing Unreal cares about from this module.
    Sam will load this file on his side and move the rover along these waypoints.
    """
    waypoints = {}
    for i, cell in enumerate(path):
        x_m, y_m = grid.cell_center_world_m(cell)
        waypoints[i] = {
            "x": x_m * UNREAL_UNITS_PER_METER,
            "y": y_m * UNREAL_UNITS_PER_METER,
            "z": 0.0
        }

    data = {"Path": waypoints}

    json_string = json.dumps(data, indent=2)
    print(f"[planner] Mission plan JSON string generated")
    return json_string

# =======================================
# INTEGRATION ENTRYPOINT (FOR MQTT WRAPPER)
# =======================================

def plan_from_vision_message(
    vision_msg: dict,
    mission_objective_labels: List[str],
    rover_label: str,
    start_soc: float,
    cost_grid: Optional[List[List[Optional[float]]]] = None,
    battery_model: Optional[BatteryModel] = None
):
    """
    This is the function your MQTT wrapper should call.

    vision_msg: dict with structure like:
        {
          "objects": [
            {"label": "rover", "x": ..., "y": ..., "w": ..., "h": ...},
            {"label": "blue",  "x": ..., "y": ..., "w": ..., "h": ...},
            ...
          ]
        }

    mission_objective_labels: e.g. ["blue", "red"]
    rover_label: "rover"
    start_soc: starting state-of-charge in percent
    mission_id: string tag for this run

    Steps:
      1) Build GridMap (with your real cost grid if provided)
      2) Convert JSON objects to BBox list
      3) Plan multi-objective mission with A* + SOC
      4) Export mission_plan.json for Unreal
    """
    # 1) Build grid
    grid = GridMap(GRID_WIDTH, GRID_HEIGHT, cost=cost_grid)

    # 2) Build BBox list from vision_msg
    bboxes: List[BBox] = []
    for obj in vision_msg.get("objects", []):
        bboxes.append(
            BBox(
                label=obj["label"],
                x=float(obj["x"]),
                y=float(obj["y"]),
                w=float(obj["w"]),
                h=float(obj["h"]),
            )
        )

    # Add battery cells based on object labels
    for b in bboxes:
        if b.label in {"blue", "red"}:   # your battery stations
            cell = pixel_bbox_to_grid_cell(b, grid)
            grid.add_battery_cell(cell.ix, cell.iy)
            print(f"[planner] Added battery station at cell {cell.ix}, {cell.iy}")

    # 3) Battery model
    if battery_model is None:
        battery_model = BatteryModel()

    # 4) Plan mission
    full_path, total_cost, final_soc = plan_mission_ordered(
        grid=grid,
        bboxes=bboxes,
        mission_objective_labels=mission_objective_labels,
        rover_label=rover_label,
        start_soc=start_soc,
        battery_model=battery_model,
    )

    print(f"[planner] Mission path length: {len(full_path)} cells")
    print(f"[planner] Total cost: {total_cost:.2f}")
    print(f"[planner] Final SOC: {final_soc:.1f}%")

    # 5) Export for Unreal
    path_json = get_path_json(
        grid=grid,
        path=full_path    )

    return path_json

def on_message(client, userdata, msg):
    global sta1Update, sta2Update, sta1Info, sta2Info
    payload = msg.payload.decode()

    if msg.topic == "Sta1":
        sta1Info = json.loads(payload)
        print(f"[Sta1] {sta1Info}")
        sta1Update = True
    elif msg.topic == "Sta2":
        sta2Info = json.loads(payload)
        print(f"[Sta2] {sta2Info}")
        sta2Update = True
    elif msg.topic == "Pos":
        objectPositions = payload
        print(f"[Pos] {objectPositions}")
        pos_updated(objectPositions)

def pos_updated(objectPositions):
    global sta1Update, sta2Update, planning
    if not (sta1Update and sta2Update) or planning:
        return
    threading.Thread(target=pos_cont, args=(objectPositions,)).start()

def pos_cont(objectPositions):
    global battery, vision_msg, sta1Update, sta2Update, sta1Info, sta2Info, path_json, planning, mqttc, MQTTTOPICPATH
    planning = True
    data = json.loads(objectPositions)

    # Step 2 — convert to planner input format
    vision_msg = {
        "objects": [
            {
                "label": label,
                "x": box["x"],
                "y": box["y"],
                "w": box["w"],
                "h": box["h"]
            }
            for label, box in data.items()
        ]
    }

    for obj in vision_msg["objects"]:
        if obj["label"] == "black":
            obj["label"] = "rover"

    print(vision_msg)

    battery = BatteryModel(station_specs={"blue": sta1Info, "red": sta2Info})

    '''path_json = plan_from_vision_message(
    vision_msg=vision_msg,
    mission_objective_labels=mission_labels,
    rover_label="rover",
    start_soc=start_soc,
    cost_grid=cost_grid,
    battery_model=battery)

    print(path_json)'''

    path_order = {"Path": find_order(vision_msg)}

    mqttc.publish(MQTTTOPICPATH, json.dumps(path_order))

    sta1Update = False
    sta2Update = False
    planning = False

# Objective, battery, x2, origin
def find_order(vision_msg):
    objects = vision_msg["objects"]

    # ---- Build corner groups ----
    def get_corners(obj):
        return [
            (obj['x'], obj['y']),
            (obj['x'] + obj['w'], obj['y']),
            (obj['x'], obj['y'] + obj['h']),
            (obj['x'] + obj['w'], obj['y'] + obj['h'])
        ]

    # Battery 1 corners: 0–3, battery 2 corners: 4–7
    bat_corners = [
        get_corners(objects[0]),
        get_corners(objects[1])
    ]

    # Objective 1: objects[2], Objective 2: [3], Objective 3: [4]
    obj_corners = [
        get_corners(objects[2]),
        get_corners(objects[3]),
    ]

    # Origin reference is OBJECT 3 (objects[4])
    origin_x = objects[4]['x']
    origin_y = objects[4]['y']

    path = {}
    step = 1

    # ---- Helper to find closest corner among a group ----
    def closest_corner(ref_x, ref_y, corners):
        best = None
        best_dist = float('inf')
        for (cx, cy) in corners:
            d = euclid_dist(ref_x, ref_y, cx, cy)
            if d < best_dist:
                best_dist = d
                best = (cx, cy)
        return best, best_dist

    # ---- Helper to find closest OBJECT among the remaining ones ----
    def closest_object(ref_x, ref_y, obj_list):
        best_idx = None
        best_corner = None
        best_dist = float('inf')

        for idx, corners in obj_list:
            corner, dist = closest_corner(ref_x, ref_y, corners)
            if dist < best_dist:
                best_idx = idx
                best_corner = corner
                best_dist = dist

        return best_idx, best_corner

    # ---- Step 1: Choose first objective ----
    remaining_objs = [(0, obj_corners[0]), (1, obj_corners[1])]  # objective 0 & 1 (skip origin)
    first_idx, first_corner = closest_object(origin_x, origin_y, remaining_objs)

    path[step] = {"x": first_corner[0], "y": first_corner[1], "z": 0.0}
    step += 1

    # Update reference point
    ref_x, ref_y = first_corner

    # ---- Step 2: Closest battery to this point ----
    bat_idx, bat_corner = closest_object(ref_x, ref_y, [(0, bat_corners[0]), (1, bat_corners[1])])

    path[step] = {"x": bat_corner[0], "y": bat_corner[1], "z": 0.0}
    step += 1

    ref_x, ref_y = bat_corner

    # ---- Step 3: Second objective (the one not chosen before) ----
    remaining_objs = [(1-first_idx, obj_corners[1-first_idx])]
    second_idx, second_corner = closest_object(ref_x, ref_y, remaining_objs)

    path[step] = {"x": second_corner[0], "y": second_corner[1], "z": 0.0}
    step += 1

    ref_x, ref_y = second_corner

    # ---- Step 4: Other battery ----
    remaining_bat = 1 - bat_idx
    other_bat_corner, _ = closest_corner(ref_x, ref_y, bat_corners[remaining_bat])

    path[step] = {"x": other_bat_corner[0], "y": other_bat_corner[1], "z": 0.0}
    step += 1

    # ---- Step 5: Return to origin ----
    path[step] = {"x": origin_x, "y": origin_y, "z": 0.0}

    return path

def euclid_dist(x1, x2, y1, y2):
    import math
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# --- Main Function ---

if __name__ == "__main__":
    planning = False
    latest = None
    battery = None
    vision_msg = None
    objectPositions = None
    sta1Info = None
    sta2Info = None
    sta1Update = False
    sta2Update = False
    cost_grid = None  
    path_json = None

    # --- Choose mission order ---
    mission_labels = ["yellow", "green"]
    battery_labels = ["blue", "red"]

    # --- Start SOC ---
    start_soc = 100.0

    MQTTHOST = "10.3.141.1"
    MQTTPORT = 1883
    MQTTTOPICSTA1 = "Sta1"
    MQTTTOPICSTA2 = "Sta2"
    MQTTTOPICPOS = "Pos"
    MQTTTOPICPATH = "Path"

    mqttc = mqtt.Client()
    mqttc.on_message = on_message
    mqttc.connect(MQTTHOST, MQTTPORT)

    mqttc.subscribe(MQTTTOPICPOS)
    mqttc.subscribe(MQTTTOPICSTA1)
    mqttc.subscribe(MQTTTOPICSTA2)

    mqttc.loop_start()

    try:
        while True:
            import time
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[planner] Exiting...")
        mqttc.loop_stop()
        mqttc.disconnect()