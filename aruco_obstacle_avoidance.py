#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Bool
from math import pi, atan2, sqrt

class ObstacleDetector:
    def __init__(self):
        # ROS setup
        rospy.init_node('obstacle_detector', anonymous=True)
        self.obstacle_pub = rospy.Publisher('/obstacle_detected', Bool, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscribe to robot's current pose
        self.current_pose = Pose2D()
        rospy.Subscriber('/robot_pose', Pose2D, self.pose_callback)
        
        # Camera calibration
        self.camera_matrix = np.array([
            [1.07139173e+03, 0.00000000e+00, 9.72592860e+02],
            [0.00000000e+00, 1.05703915e+03, 6.36955649e+02],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ], dtype=np.float32)

        self.dist_coeffs = np.array([[0.31466559, -1.09088039, 0.03388564, -0.00394274, 2.31947725]])
        
        # Scaling 
        scale_factor = 0.009
        self.camera_matrix[0,0] *= scale_factor 
        self.camera_matrix[1,1] *= scale_factor

        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Video capture
        self.cap = cv2.VideoCapture("http://your_phone_ip:8080/video")
        
        # Navigation states
        self.STATE_NORMAL = 0
        self.STATE_CIRCLING = 1
        self.current_state = self.STATE_NORMAL
        
        # Circling parameters
        self.circle_start_time = None
        self.circle_duration = 8  # seconds to complete circle
        self.circle_radius = 0.5  # meters
        self.obstacle_detected = False
        self.marker_center = None
        self.marker_distance = None
        
        # Command message
        self.cmd_vel_msg = Twist()
        
        # Get obstacle IDs
        self.get_obstacle_ids()

    def pose_callback(self, msg):
        self.current_pose = msg

    def get_obstacle_ids(self):
        while not rospy.is_shutdown():
            try:
                obstacle_ids_input = '1,5,7'
                self.obstacle_ids = [int(id.strip()) for id in obstacle_ids_input.split(',')]
                if not self.obstacle_ids:
                    raise ValueError
                break
            except ValueError:
                rospy.logerr("Invalid input! Please enter comma-separated numbers (e.g., '1,5,7')")

    def draw_dashed_line(self, img, pt1, pt2, color, thickness=1, dash_length=10):
        dist = int(np.hypot(pt2[0]-pt1[0], pt2[1]-pt1[1]))
        for i in range(0, dist, dash_length*2):
            start = (int(pt1[0] + (pt2[0]-pt1[0])*i/dist), int(pt1[1] + (pt2[1]-pt1[1])*i/dist))
            end = (int(pt1[0] + (pt2[0]-pt1[0])*(i+dash_length)/dist), int(pt1[1] + (pt2[1]-pt1[1])*(i+dash_length)/dist))
            cv2.line(img, start, end, color, thickness)

    def calculate_circle_velocity(self, marker_center, marker_distance):
        # Calculate circular motion
        if self.circle_start_time is None:
            self.circle_start_time = rospy.Time.now()
        
        elapsed_time = (rospy.Time.now() - self.circle_start_time).to_sec()
        
        if elapsed_time < self.circle_duration:
            # Create circular motion
            # Adjust linear and angular velocities based on marker distance
            base_linear_vel = 0.2
            base_angular_vel = 2 * pi / self.circle_duration
            
            # Scale velocities based on distance
            distance_scale = min(marker_distance / 0.8, 1.0)  # 0.8m is the reference distance
            
            cmd = Twist()
            cmd.linear.x = base_linear_vel * distance_scale
            cmd.angular.z = base_angular_vel
            
            return cmd, False
        else:
            # Finished circling
            self.circle_start_time = None
            return Twist(), True

    def run(self):
        rate = rospy.Rate(30)  # 30Hz
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("Frame capture error")
                break

            # Detect ArUco markers
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            self.obstacle_detected = False
            if ids is not None:
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    if marker_id in self.obstacle_ids:
                        corner = corners[i]
                        cv2.aruco.drawDetectedMarkers(frame, [corner], ids[i:i+1])
                        
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, 0.05, self.camera_matrix, self.dist_coeffs)
                        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)
                        
                        distance = np.linalg.norm(tvec)
                        angle = np.degrees(np.arctan2(tvec[0][0][0], tvec[0][0][2]))
                        center = tuple(np.mean(corner[0], axis=0).astype(int))
                        
                        cv2.putText(frame, f"ID:{marker_id} {distance:.2f}m {angle:.2f}°", 
                                   (center[0]+10, center[1]-10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                        self.draw_dashed_line(frame, (frame.shape[1]//2, frame.shape[0]//2), center, (0,255,0), 2)
                        
                        # Check if obstacle is too close
                        if distance < 0.8:  # 0.8 meters threshold
                            self.obstacle_detected = True
                            self.marker_center = center
                            self.marker_distance = distance

            # State machine for navigation
            if self.current_state == self.STATE_NORMAL:
                if self.obstacle_detected:
                    # Switch to circling state
                    self.current_state = self.STATE_CIRCLING
                    self.circle_start_time = None
                    rospy.loginfo("Obstacle detected! Starting circling maneuver")
                else:
                    # No obstacle - let Go_To_Goal handle navigation
                    self.cmd_vel_msg = Twist()  # Zero velocity to let Go_To_Goal take control
                    
            elif self.current_state == self.STATE_CIRCLING:
                if self.obstacle_detected:
                    # Continue circling
                    self.cmd_vel_msg, finished_circle = self.calculate_circle_velocity(
                        self.marker_center, self.marker_distance)
                    if finished_circle:
                        self.current_state = self.STATE_NORMAL
                        rospy.loginfo("Finished circling, returning to normal navigation")
                else:
                    # Lost sight of obstacle, return to normal
                    self.current_state = self.STATE_NORMAL
                    self.cmd_vel_msg = Twist()
                    rospy.loginfo("Lost obstacle, returning to normal navigation")

            # Publish obstacle detection status
            self.obstacle_pub.publish(Bool(data=self.obstacle_detected))
            
            # Publish velocity command if in circling state
            if self.current_state == self.STATE_CIRCLING:
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
            
            # Display state
            state_text = "Circling" if self.current_state == self.STATE_CIRCLING else "Normal"
            cv2.putText(frame, f"State: {state_text}", (20, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(frame, f"Obstacle: {'Yes' if self.obstacle_detected else 'No'}", (20, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            
            cv2.imshow("Obstacle Detection", frame)
            if cv2.waitKey(1) == ord('q'):
                break
            
            rate.sleep()

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector = ObstacleDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
