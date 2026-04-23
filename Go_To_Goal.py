#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, pi, degrees, radians, cos, sin
import tf
from std_msgs.msg import Bool


class RobotController:

    def __init__(self):
        rospy.init_node('go_to_goal_controller', anonymous=True)

        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.stop_publisher = rospy.Publisher('stop_motors', Bool, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('robot_pose', Pose2D, self.update_pose)
        self.obstacle_detected = False
        rospy.Subscriber('/obstacle_detected', Bool, self.obstacle_callback)

        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.distance_tolerance = 0.2 # meters
        self.angle_tolerance = radians(10)  # radians
        self.max_linear_speed = 0.3  # m/s (reduced for better control)
        self.min_linear_speed = 0.1  # m/s
        self.max_angular_speed = 0.5  # rad/s (reduced for better control)
        self.min_angular_speed = 0.1  # rad/s
        self.rate = rospy.Rate(10)  # 10 Hz
        
        # Wait for pose updates to start
        rospy.sleep(1)

    def update_pose(self, data):
        """Callback for Pose2D message."""
        self.pose['x'] = round(data.x, 4)
        self.pose['y'] = round(data.y, 4)
        self.pose['theta'] = data.theta
    
    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data

    def normalize_angle(self, angle):
        """Normalize angle to be between -pi and pi."""
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def euclidean_distance(self, goal):
        """Calculate distance to goal."""
        return sqrt(pow(goal['x'] - self.pose['x'], 2) + 
                   pow(goal['y'] - self.pose['y'], 2))

    def steering_angle(self, goal):
        """Calculate angle to goal."""
        return self.normalize_angle(atan2(goal['y'] - self.pose['y'], 
                                        goal['x'] - self.pose['x']))

    def angular_difference(self, goal_angle):
        """Calculate the shortest angular difference."""
        current = self.normalize_angle(self.pose['theta'])
        target = self.normalize_angle(goal_angle)
        diff = self.normalize_angle(target - current)
        return diff

    def linear_vel(self, distance, angle_diff):
        """Calculate linear velocity with smooth deceleration and angle consideration."""
        # Reduce speed when turning sharply
        angle_factor = cos(angle_diff)  # Will be 1 when aligned, 0 when perpendicular
        if abs(angle_diff) > pi/4:  # If angle difference is more than 45 degrees
            return 0  # Stop and turn first
            
        if distance < 0.5:  # Start slowing down within 0.5m
            vel = self.min_linear_speed + (self.max_linear_speed - self.min_linear_speed) * (distance / 0.5)
            vel = max(self.min_linear_speed, min(vel, self.max_linear_speed))
        else:
            vel = self.max_linear_speed
            
        return vel * max(0, angle_factor)  # Scale by angle factor

    def angular_vel(self, angle_diff):
        """Calculate angular velocity with smooth control."""
        if abs(angle_diff) < radians(30):  # Start slowing down within 30 degrees
            vel = self.min_angular_speed + (self.max_angular_speed - self.min_angular_speed) * (abs(angle_diff) / radians(30))
            vel = max(self.min_angular_speed, min(vel, self.max_angular_speed))
        else:
            vel = self.max_angular_speed
        return vel if angle_diff > 0 else -vel

    def stop_robot(self):
        """Send stop commands to the robot."""
        # Send zero velocity
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        
        # Send stop signal
        stop_msg = Bool()
        stop_msg.data = True
        self.stop_publisher.publish(stop_msg)
        rospy.sleep(0.1)  # Make sure the message is sent
        
    def move2goal(self):
        """Move robot to goal with improved control."""
        goal = {
            'x': float(input("Enter x goal position: ")),
            'y': float(input("Enter y goal position: "))
        }
        
        print(f"\nMoving to goal: ({goal['x']:.2f}, {goal['y']:.2f})")
        print(f"Current position: ({self.pose['x']:.2f}, {self.pose['y']:.2f})")
        
        vel_msg = Twist()
        last_angle_diff = 0
        
        while not rospy.is_shutdown():
            distance = self.euclidean_distance(goal)
            goal_angle = self.steering_angle(goal)
            angle_diff = self.angular_difference(goal_angle)
            
            # Detect if we're spinning due to angle wraparound
            if abs(angle_diff - last_angle_diff) > pi:
                print("\nDetected possible angle wraparound, adjusting...")
                angle_diff = last_angle_diff  # Use previous angle difference
            
            last_angle_diff = angle_diff
            
            # Check if we've reached the goal
            if distance < self.distance_tolerance:
                print("\nReached the goal!")
                self.stop_robot()
                break
            
            # Calculate velocities
            angular_velocity = self.angular_vel(angle_diff)
            linear_velocity = self.linear_vel(distance, angle_diff)
            
            # Set and publish velocities
            vel_msg.linear.x = linear_velocity
            vel_msg.angular.z = angular_velocity
            self.velocity_publisher.publish(vel_msg)
            
            # Debug output
            print(f"\rDistance: {distance:.3f}m, Angle diff: {degrees(angle_diff):.1f}°, "
                  f"Linear vel: {linear_velocity:.2f}, Angular vel: {angular_velocity:.2f}", end='')
            
            self.rate.sleep()
        
        # Final stop
        self.stop_robot()
        print("\nRobot stopped")


if __name__ == '__main__':
    try:
        controller = RobotController()
        while not rospy.is_shutdown():
            controller.move2goal()
            if input("\nDo you want to set a new goal? (y/n): ").lower() != 'y':
                controller.stop_robot()  # Make sure robot stops before exiting
                break
    except rospy.ROSInterruptException:
        pass