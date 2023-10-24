import cv2
import cv2.aruco as aruco

parameters = aruco.DetectorParameters_create()

# Load the dictionary, 4x4_50 is what we used according to rule book
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

# cv2.CAP_V4L was needed to run on my machine, might need to change for rover
cap = cv2.VideoCapture(0, cv2.CAP_V4L)

while True:
    ret, frame = cap.read()

    # Detects markers in the image
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    # If markers detected
    if len(corners) > 0:
        frame = aruco.drawDetectedMarkers(frame, corners, ids)

    cv2.imshow('frame', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

