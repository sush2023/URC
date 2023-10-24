import cv2
import cv2.aruco as aruco

parameters = aruco.DetectorParameters_create()

# Load the dictionary, 4x4_50 is what we use according to rule book
arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_50)

# cv2.CAP_V4L says to use Video4linux2 driver, might be different for jetson
capObj = cv2.VideoCapture(0, cv2.CAP_V4L)

while True:
    returnedValue, frame = capObj.read()

    # Detects markers in the image
    corners, ids, rejectedTags = aruco.detectMarkers(frame, arucoDict, parameters=parameters)

    # If markers detected
    if len(corners) > 0:
        frame = aruco.drawDetectedMarkers(frame, corners, ids)

    cv2.imshow('frame', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

capObj.release()
cv2.destroyAllWindows()

