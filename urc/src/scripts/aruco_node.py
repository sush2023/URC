#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import cv2
import cv2.aruco as aruco
import math

# Initialize the ROS node
rospy.init_node('aruco_detector')

# Create a publisher for the tag id and distance
pub = rospy.Publisher('aruco_distance', String, queue_size=10)

parameters = aruco.DetectorParameters_create()
arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_50)
capObj = cv2.VideoCapture(0, cv2.CAP_V4L)

actual_tag_size = 0.095
focal_length = 1025

while not rospy.is_shutdown():
    returnedValue, frame = capObj.read()
    corners, ids, rejectedTags = aruco.detectMarkers(frame, arucoDict, parameters=parameters)

    if len(corners) > 0:
        frame = aruco.drawDetectedMarkers(frame, corners, ids)

        for i in range(len(corners)):
            dx = corners[i][0][0][0] - corners[i][0][2][0]
            dy = corners[i][0][0][1] - corners[i][0][2][1]
            pixel_tag_size = math.sqrt(dx**2 + dy**2)

            distance = actual_tag_size * focal_length / pixel_tag_size

            # Publish the tag id and distance as a string message
            msg = f"Tag id: {ids[i][0]}, Distance: {distance} meters"
            pub.publish(msg)

    cv2.imshow('frame', frame)
if cv2.waitKey(1) & 0xFF == ord('q'):
        break

capObj.release()
cv2.destroyAllWindows()

