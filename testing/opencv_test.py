import cv2
import numpy as np
import math

# start video capture
video = cv2.VideoCapture(0)

while True:

    # current frame
    _, frame = video.read()

    ### Image Processing ###


    # color constants
    blue = (255, 0, 0)
    red = (0, 0, 255)
    green = (0, 255, 0)
    white = (255, 255, 255)
    black = (0, 0, 0)

    height, width = frame.shape[0:2]
    image_center = (int(width/2), int(height/2))
    # print("height:", height, "width:", width)
    # Image height: 480, Image width: 640

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # (H, S, V)
    # blue object
    lower = np.array([80, 155, 20])
    higher = np.array([130, 255, 255])

    mask = cv2.inRange(hsv, lower, higher)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # if we want to get contours within a certain size
    # allcontours_inrange = []
    # for contour in contours:
    #     area = cv2.contourArea(contour)
    #     if 2000 < area < 100000:
    #         allcontours_inrange.append(contour)
    # area = cv2.contourArea(c)
    # print(area)


    if len(contours) != 0:

        # only get the biggest contour if there are multiple
        c = max(contours, key=cv2.contourArea)

        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(frame, (x, y), (x + w, y + h), blue, 3)

        # get midpoint of rectangle and print coordinates
        # (x,y) top left, (x+w, y+w) top right
        mid_x = x + w/2
        mid_y = y + h/2
        object_center = (int(mid_x), int(mid_y))

        text1 = f"Center of rectangle: ({mid_x}, {mid_y})"
        cv2.putText(frame, text1, (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, black, 1)
        cv2.circle(frame, object_center, 2, green, 5)

        # display circle at center and line from image center to object center
        distance = math.sqrt((object_center[0]-image_center[0]) ** 2 + (object_center[1]-image_center[1]) ** 2)
        text2 = f"Distance: {distance}"
        cv2.line(frame, object_center, image_center, red, 3)
        cv2.putText(frame, text2, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, black, 1)

    # Display the frame image with bounded rectangle and masked image
    cv2.imshow('Video', frame)
    # cv2.imshow('Mask', mask)

    # Detect if the Esc key has been pressed
    c = cv2.waitKey(1)
    if c == 27:
        break

# Release the video capture object & close all active windows
video.release()
cv2.destroyAllWindows()
