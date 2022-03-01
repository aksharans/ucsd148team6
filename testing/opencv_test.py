import cv2
import numpy as np


#Initialize video capture
video = cv2.VideoCapture(0)

while True:
    # Capture the current frame
    _, frame = video.read()

    # height, width = frame.shape[0:2]
    # print("height:", height, "width:", width)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([40, 50, 20])
    higher = np.array([75, 255, 255])

    mask = cv2.inRange(hsv, lower, higher)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) != 0:
        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(frame, (x,y), (x + w, y + h), (0, 255, 0), 3)

        mid_x = x/2
        mid_y = y/2
        text = f"Center of rectangle: ({mid_x}, {mid_y})"
        cv2.putText(frame, text, (200, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))

    # Display the image
    cv2.imshow('Video', frame)
    cv2.imshow('Mask', mask)

    # Detect if the Esc key has been pressed
    c = cv2.waitKey(1)
    if c == 27:
        break
# Release the video capture object
video.release()
# Close all active windows
cv2.destroyAllWindows()