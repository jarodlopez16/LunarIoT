import math
import itertools
import json
from pathlib import Path
from dataclasses import dataclass
from typing import List, Tuple, Optional

# Geometry and motion
ROVER_SPEED_MPS = 0.20 # I set rover speed in meters per second

# Battery pack (example: 2S Li-ion, 7 point 4 V nominal, 5 Ah => 37 Wh)
BATTERY_VOLTAGE_V = 7.4 # I store nominal voltage in volts
BATTERY_CAPACITY_AH = 5.0 # I store capacity in amp hours
BATTERY_CAPACITY_WH = BATTERY_VOLTAGE_V * BATTERY_CAPACITY_AH
BATTERY_CAPACITY_J = BATTERY_CAPACITY_WH * 3600.0 # watt seconds (joules)

# Electrical load while driving (rough estimate)
DRIVE_CURRENT_A = 2.0 # I store average current while driving
DRIVE_POWER_W = BATTERY_VOLTAGE_V * DRIVE_CURRENT_A

# Charger (example numbers, tune when you measure real charge current)
CHARGER_CURRENT_A = 1.0 # I store constant current during charge
CHARGER_POWER_W = BATTERY_VOLTAGE_V * CHARGER_CURRENT_A

# Safety margin
MIN_SOC_RESERVE = 10.0 # percent, I do not drop below this in any leg

# ========================
# Core data structures
# =========================

@dataclass
class Node:
# """
# This represents one point on the map.

# name: label for plots or logs
# x, y: coordinates in meters in the mission frame
# is_battery: True when this node is a charging station
# """

name: str
x: float
y: float
is_battery: bool = False

@dataclass
class RoverState:
# """
# This stores rover battery state as SOC in percent.
# """
soc: float


# =========================
# Low level models
# =========================

def distance(a: Node, b: Node) -> float:
# """
# This returns Euclidean distance between two nodes in meters.
# """

dx = b.x - a.x
dy = b.y - a.y
return math.hypot(dx, dy)


def drive_leg_time_and_soc(
start_state: RoverState,
start: Node,
end: Node,
) -> Tuple[float, RoverState]:
# """
# This models one drive leg from start to end.

# Time:
# I move at constant ROVER_SPEED_MPS.

# Energy:
# I assume constant electrical power DRIVE_POWER_W while moving.

# energy_used = DRIVE_POWER_W * time

# I convert this energy to a percent SOC drop using BATTERY_CAPACITY_J.

# This is where I later plug in PyBaMM:
# - Instead of DRIVE_POWER_W, I call a PyBaMM model.
# - I use PyBaMM to compute delta_SOC for this leg.
# """
d = distance(start, end)
time_sec = d / ROVER_SPEED_MPS

# Energy used in joules
energy_used_j = DRIVE_POWER_W * time_sec

# I convert energy to SOC drop
soc_drop = 100.0 * (energy_used_j / BATTERY_CAPACITY_J)
new_soc = max(0.0, start_state.soc - soc_drop)

return time_sec, RoverState(new_soc)


def charge_at_station_time_and_soc(
start_state: RoverState,
station: Node,
target_soc: float = 100.0,
) -> Tuple[float, RoverState]:
# """
# This models charging at a station.

# Time:
# I charge with constant current CHARGER_CURRENT_A.

# Energy:
# I compute how many joules correspond to the missing SOC,
# then divide by CHARGER_POWER_W to get time.

# This is also where I later plug in PyBaMM to replace the
# constant current approximation.
# """
if start_state.soc >= target_soc:
return 0.0, start_state

soc_missing = target_soc - start_state.soc
energy_needed_j = (soc_missing / 100.0) * BATTERY_CAPACITY_J

time_sec = energy_needed_j / CHARGER_POWER_W
new_soc = target_soc

return time_sec, RoverState(new_soc)


# =========================
# Path level simulation
# =========================

def path_cost_with_battery(
start_state: RoverState,
path: List[Node],
verbose: bool = False,
) -> Tuple[float, bool]:
# """
# This simulates a full path and returns:

# total_time_seconds, feasible_flag

# Feasible means:
# SOC never drops below MIN_SOC_RESERVE.
# """
total_time = 0.0
state = RoverState(start_state.soc)

if verbose:
print(f"Start at {path[0].name} with SOC = {state.soc:.1f}%")

for i in range(len(path) - 1):
a = path[i]
b = path[i + 1]

# I drive from a to b
drive_time, state = drive_leg_time_and_soc(state, a, b)
total_time += drive_time

if verbose:
print(
f"Drive {a.name} -> {b.name}: "
f"time = {drive_time:.1f}s, SOC = {state.soc:.1f}%"
)

# I enforce reserve
if state.soc < MIN_SOC_RESERVE:
if verbose:
print(
f"Path infeasible: SOC = {state.soc:.1f}% "
f"is below reserve {MIN_SOC_RESERVE:.1f}%"
)
return float("inf"), False

# If I arrive at a battery station, I charge
if b.is_battery:
charge_time, state = charge_at_station_time_and_soc(state, b)
total_time += charge_time
if verbose:
print(
f"Charge at {b.name}: "
f"time = {charge_time:.1f}s, SOC = {state.soc:.1f}%"
)

if verbose:
print(f"Total time for this path = {total_time:.1f}s")
return total_time, True


# =========================
# Search over permutations
# =========================

def generate_node_sets(
destinations: List[Node],
batteries: List[Node],
) -> List[List[Node]]:
# """
# This generates all node sets that include all destinations
# and any subset of batteries.

# Each battery appears at most once in a path.
# """
sets: List[List[Node]] = []
for r in range(len(batteries) + 1):
for battery_subset in itertools.combinations(batteries, r):
node_set = list(destinations) + list(battery_subset)
sets.append(node_set)
return sets


def find_best_route(
start_node: Node,
end_node: Optional[Node],
start_state: RoverState,
destinations: List[Node],
batteries: List[Node],
) -> Tuple[List[Node], float]:
# """
# This searches all reasonable paths and returns the best one.

# I do:
# - every subset of batteries
# - every permutation of destinations + chosen batteries

# For each candidate path:
# - I simulate battery and time with path_cost_with_battery
# - I discard infeasible paths
# - I keep the path with the minimum total time

# This brute force search is acceptable when the number of
# nodes is small, which matches our mission profile.
# """
best_path: Optional[List[Node]] = None
best_cost: float = float("inf")

for node_set in generate_node_sets(destinations, batteries):
for perm in itertools.permutations(node_set):
path = [start_node] + list(perm)
if end_node is not None:
path.append(end_node)

cost, feasible = path_cost_with_battery(start_state, path, verbose=False)
if feasible and cost < best_cost:
best_cost = cost
best_path = path

if best_path is None:
return [start_node], float("inf")
return best_path, best_cost


# =========================
# Unity integration
# =========================

def export_path_to_json(path: List[Node], file_path: str = "mission_plan.json") -> None:
# """
# This exports the ordered list of waypoints for Unity.

# I map:
# Node.x -> Unity world X
# Node.y -> Unity world Z
# Y is set to 0 point 0 by default for a flat floor.

# Unity reads this file from StreamingAssets and uses its own A star
# and NavMesh to move the rover between these waypoints.
# """
waypoints = []
for node in path:
waypoints.append(
{
"name": node.name,
"x": node.x, # Unity X
"y": 0.0, # Unity Y height, I keep it 0 point 0 for now
"z": node.y, # Unity Z
}
)

data = {
"mission_id": "run_001",
"waypoints": waypoints,
}

out_path = Path(file_path)
with out_path.open("w") as f:
json.dump(data, f, indent=2)

print(f"Mission plan written to {out_path.absolute()}")


# =========================
# Demo scenario (replace coordinates)
# =========================

def demo_scenario():
# """
# This runs a demo with three destinations and two battery stations.

# These coordinates must be replaced with real mission coordinates
# from Unity. I use them here only as an example.
# """
# Base at origin
base = Node("Base", 0.0, 0.0)

# Destinations (science / objectives)
d1 = Node("D1", 2.0, 1.0)
d2 = Node("D2", -1.0, 3.0)
d3 = Node("D3", 4.0, 4.0)

# Battery stations
b1 = Node("B1", 1.0, 2.0, is_battery=True)
b2 = Node("B2", -2.0, 0.5, is_battery=True)

# I start with 30 percent SOC to force a choice about charging
start_state = RoverState(soc=30.0)

best_path, best_cost = find_best_route(
start_node=base,
end_node=base,
start_state=start_state,
destinations=[d1, d2, d3],
batteries=[b1, b2],
)

print("Best path (names only):")
for n in best_path:
print(" -", n.name)
print(f"Total time: {best_cost:.1f} seconds\n")

print("Detailed simulation for best path:")
path_cost_with_battery(start_state, best_path, verbose=True)

# I also export this best path for Unity
export_path_to_json(best_path, "mission_plan.json")


if __name__ == "__main__":
demo_scenario()