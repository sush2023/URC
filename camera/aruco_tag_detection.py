import cv2
import cv2.aruco as aruco
import math

parameters = aruco.DetectorParameters_create()
arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_50)
capObj = cv2.VideoCapture(0, cv2.CAP_V4L)

# size of the ArUco tag in meters (0.1m = 10cm), should be changed to 0.1, I set it to 0.095 becuase my test paper is to small
actual_tag_size = 0.095

# idk I just guess and checked till it worked, come back to this
focal_length = 1025

while True:
    returnedValue, frame = capObj.read()
    corners, ids, rejectedTags = aruco.detectMarkers(frame, arucoDict, parameters=parameters)

    if len(corners) > 0:
        frame = aruco.drawDetectedMarkers(frame, corners, ids)

        # For each detected tag
        for i in range(len(corners)):
            # Calculate the size of the tag in pixels
            dx = corners[i][0][0][0] - corners[i][0][2][0]
            dy = corners[i][0][0][1] - corners[i][0][2][1]
            pixel_tag_size = math.sqrt(dx**2 + dy**2)

            distance = actual_tag_size * focal_length / pixel_tag_size

            # Print out the id and distance of each tag
            print(f"Tag id: {ids[i][0]}, Distance: {distance} meters")

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

capObj.release()
cv2.destroyAllWindows()
