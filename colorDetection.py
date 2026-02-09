import numpy as np
import cv2
import json
import paho.mqtt.client as mqtt
import time

MQTTHOST = "10.3.141.1"
MQTTPORT = 1883
MQTTTOPIC = "Pos"

lastPub = 0
latestBluePos = None
latestRedPos = None
latestGreenPos = None
latestYellowPos = None
latestBlackPos = None

url = "http://10.3.141.171:81/stream"
capture = cv2.VideoCapture(url)

fourcc = cv2.VideoWriter_fourcc(*"mp4v")  
fps = 20                                   
out = cv2.VideoWriter("demonstration.mp4", fourcc, fps, (1280, 720))

mqttc = mqtt.Client()
mqttc.connect(MQTTHOST, MQTTPORT)
mqttc.loop_start()

def publishInterval(lastPub, client, topic, message, interval=0.5):
    now = time.time()
    if now - lastPub >= interval:
        client.publish(topic, message)
        return now
    return lastPub

while True:
    ret, frame = capture.read() 
    if not ret or frame is None:
        print("Failed to read frame")
        continue

    width = int(capture.get(3))
    height = int(capture.get(4))

    kernel = np.ones((5, 5), "uint8")
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # Convert pixels to hue, saturation, and lightness

    lower_blue = np.array([94, 80, 2])
    upper_blue = np.array([126, 255, 255])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue) # Only keep the blue pixels
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)

    # Two upper and lowers because red wraps from 0 to 180
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    red_mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2) # Only keep the red pixels
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

    lower_green = np.array([35, 80, 20])
    upper_green = np.array([85, 255, 150])
    green_mask = cv2.inRange(hsv, lower_green, upper_green) # Only keep the green pixels
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

    lower_yellow = np.array([20, 40, 160])
    upper_yellow = np.array([35, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow) # Only keep the yellow pixels
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])
    white_mask = cv2.inRange(hsv, lower_white, upper_white) # Only keep the white pixels
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)

    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 50])
    black_mask = cv2.inRange(hsv, lower_black, upper_black) # Only keep the black pixels
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)

    # Results from masks
    blue_mask = cv2.dilate(blue_mask, kernel)
    result_blue = cv2.bitwise_and(frame, frame, mask=blue_mask)

    red_mask = cv2.dilate(red_mask, kernel)
    result_red = cv2.bitwise_and(frame, frame, mask=red_mask)

    green_mask = cv2.dilate(green_mask, kernel)
    result_green = cv2.bitwise_and(frame, frame, mask=green_mask)

    yellow_mask = cv2.dilate(yellow_mask, kernel)
    result_yellow = cv2.bitwise_and(frame, frame, mask=yellow_mask)

    white_mask = cv2.dilate(white_mask, kernel)
    result_white = cv2.bitwise_and(frame, frame, mask=white_mask)

    # Blue outline
    blue_contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in blue_contours:
        if cv2.contourArea(contour) < 20000 and cv2.contourArea(contour) > 3000:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(frame, "Blue", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            '''if latestBluePos == None:
                latestBluePos = {"x": x, "y": y, "w": w, "h": h}'''
            latestBluePos = {"x": x, "y": y, "w": w, "h": h}

    # Red outline
    red_contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in red_contours:
        if cv2.contourArea(contour) < 10000 and cv2.contourArea(contour) > 3000:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, "Red", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            '''if latestRedPos == None:
                latestRedPos = {"x": x, "y": y, "w": w, "h": h}'''
            latestRedPos = {"x": x, "y": y, "w": w, "h": h}

    # Green outline  
    green_contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in green_contours:
        if cv2.contourArea(contour) < 10000 and cv2.contourArea(contour) > 2500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "Green", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            '''if latestGreenPos == None:
                latestGreenPos = {"x": x, "y": y, "w": w, "h": h}'''
            latestGreenPos = {"x": x, "y": y, "w": w, "h": h}

    # Yellow outline 
    yellow_contours, hierarchy = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in yellow_contours:
        if cv2.contourArea(contour) < 10000 and cv2.contourArea(contour) > 4000:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(frame, "Yellow", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            '''if latestYellowPos == None:
                latestYellowPos = {"x": x, "y": y, "w": w, "h": h}'''
            latestYellowPos = {"x": x, "y": y, "w": w, "h": h}

    # Black outline
    black_contours, hierarchy = cv2.findContours(black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in black_contours:
        if cv2.contourArea(contour) > 5000:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 0), 2)
            cv2.putText(frame, "Black", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
            latestBlackPos = {"x": x, "y": y, "w": w, "h": h}
        #else:
        #    latestBlackPos = None

    positions = {"blue": latestBluePos, "red": latestRedPos, "green": latestGreenPos, "yellow": latestYellowPos, "black": latestBlackPos}
    positionsJSONStr = json.dumps(positions)
    lastPub = publishInterval(lastPub, mqttc, MQTTTOPIC, positionsJSONStr)

    out.write(frame)
    cv2.imshow('Colors', frame)

    if cv2.waitKey(1) == ord('q'): 
        break  

out.release()
capture.release()
mqttc.loop_stop()
cv2.destroyAllWindows()