from __future__ import print_function 
import cv2 
import numpy as np 
import rospy
from geometry_msgs.msg import Twist


desired_aruco_dictionary = "DICT_4X4_1000"

ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}
  
def main():
  if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
    print("[INFO] ArUCo tag of '{}' is not supported".format(
      args["type"]))
    sys.exit(0)
     
  print("[INFO] detecting '{}' markers...".format(
    desired_aruco_dictionary))
  this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[desired_aruco_dictionary])
  this_aruco_parameters = cv2.aruco.DetectorParameters_create()
  cap = cv2.VideoCapture("rtsp://admin:admin@192.168.0.132")
   
  while(True):
    marker_dict = {}
    positions = []
    ret, frame = cap.read()  
    
    (corners, ids, rejected) = cv2.aruco.detectMarkers(
      frame, this_aruco_dictionary, parameters=this_aruco_parameters)
       
    if len(corners) > 0:

      ids = ids.flatten()
      for (marker_corner, marker_id) in zip(corners, ids):
       
        # Extract the marker corners
        corners = marker_corner.reshape((4, 2))
        (top_left, top_right, bottom_right, bottom_left) = corners
         
        # Convert the (x,y) coordinate pairs to integers
        top_right = (int(top_right[0]), int(top_right[1]))
        bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
        bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
        top_left = (int(top_left[0]), int(top_left[1]))
         
        # Draw the bounding box of the ArUco detection
        cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
        cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
        cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
        cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)
         
        # Calculate and draw the center of the ArUco marker
        center_x = int((top_left[0] + bottom_right[0]) / 2.0)
        center_y = int((top_left[1] + bottom_right[1]) / 2.0)
        cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
         
        # Draw the ArUco marker ID on the video frame
        # The ID is always located at the top_left of the ArUco marker
        cv2.putText(frame, str(marker_id), 
          (top_left[0], top_left[1] - 15),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.5, (0, 255, 0), 2)

        dist_sq = (640-center_x)**2 + (720-center_y)**2
        positions.append([(640-center_x), (720-center_y)])
        start_point = (640, 720)
        end_point = (center_x, center_y)
        cv2.putText(frame, str(dist_sq), (top_left[0]- 50, top_left[1] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.line(frame, start_point, end_point, (255, 0, 0), 5)
        marker_dict[f"{marker_id}"] = dist_sq
    # Display the resulting frame
    if (len(positions) == 2):
        avg(positions)
    cv2.imshow('frame',frame)
    if (len(marker_dict) >= 2):
      #print(int(list(marker_dict.values())[0]))
      if (abs(int(list(marker_dict.values())[0]) - int(list(marker_dict.values())[1])) < 5000):
        print(f"{list(marker_dict.values())}")
        # send twist message

    # If "q" is pressed on the keyboard, 
    # exit this loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
  
  # Close down the video stream
  cap.release()
  cv2.destroyAllWindows()

def avg(p):
    if p[0][0] > p[1][0]:
        p[0][0], p[1][0] = p[1][0], p[0][0]

    if ((abs(p[1][0]) - abs(p[0][0]) <= 69) and (abs(p[1][0]) - abs(p[0][0]) >= -69)) and p[1][0] > 0 and p[0][0] < 0:
        print("go", " ", p[0][0], " ",p[1][0])
        move_cmd = Twist()
        move_cmd.linear.x = 1
        move_cmd.angular.z = 0

    elif (abs(p[1][0]) - abs(p[0][0]) >= 69):
        print("Turn Left", " ", p[0][0], " ",p[1][0])
        move_cmd = Twist()
        move_cmd.linear.x = 1
        move_cmd.angular.z = 1

    elif (abs(p[1][0]) - abs(p[0][0]) <= -69):
        print("Turn Right", " ", p[0][0], " ",p[1][0])
        move_cmd = Twist()
        move_cmd.linear.x = 1
        move_cmd.angular.z = -1

if __name__ == '__main__':
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub.publish(move_cmd)
    rospy.spin()
    print(__doc__)
    main()