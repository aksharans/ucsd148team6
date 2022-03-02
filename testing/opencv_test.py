import cv2
import numpy as np


# start video capture
video = cv2.VideoCapture(0)

while True:

    # current frame
    _, frame = video.read()

    # height, width = frame.shape[0:2]
    # print("height:", height, "width:", width)
    # Image height: 480, Image width: 640

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # (H, S, V)
    # green object
    lower = np.array([40, 50, 20])
    higher = np.array([75, 255, 255])

    mask = cv2.inRange(hsv, lower, higher)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) != 0:

        # only get the biggest contour if there are multiple
        c = max(contours, key=cv2.contourArea)

        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)

        # get midpoint of rectangle and print coordinates
        # (x,y) top left, (x+w, y+w) top right
        mid_x = x + w/2
        mid_y = y + h/2
        text = f"Center of rectangle: ({mid_x}, {mid_y})"
        cv2.putText(frame, text, (200, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255))

    # Display the frame image with bounded rectangle and masked image
    cv2.imshow('Video', frame)
    cv2.imshow('Mask', mask)

    # Detect if the Esc key has been pressed
    c = cv2.waitKey(1)
    if c == 27:
        break

# Release the video capture object & close all active windows
video.release()
cv2.destroyAllWindows()
