#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D, Twist
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# Data buffers
x_data, y_data, theta_data, t_data = [], [], [], []
obstacle_times = []

start_time = time.time()

def pose_callback(msg):
    current_time = time.time() - start_time
    t_data.append(current_time)
    x_data.append(msg.x)
    y_data.append(msg.y)
    theta_data.append(msg.theta)

def cmd_vel_callback(msg):
    if msg.linear.y != 0:  # Use linear.y as obstacle indicator
        current_time = time.time() - start_time
        obstacle_times.append(current_time)

def animate(i):
    plt.cla()

    plt.subplot(3, 1, 1)
    plt.plot(t_data, x_data, label="X")
    for t in obstacle_times:
        plt.axvline(x=t, color='r', linestyle='--', alpha=0.5, label='Obstacle')
    plt.ylabel("X Position")
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(t_data, y_data, label="Y", color='orange')
    for t in obstacle_times:
        plt.axvline(x=t, color='r', linestyle='--', alpha=0.5)
    plt.ylabel("Y Position")
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(t_data, theta_data, label="Theta", color='green')
    for t in obstacle_times:
        plt.axvline(x=t, color='r', linestyle='--', alpha=0.5)
    plt.xlabel("Time (s)")
    plt.ylabel("Theta (rad)")
    plt.grid(True)

def main():
    rospy.init_node('pose_plotter_with_obstacles', anonymous=True)
    rospy.Subscriber("/robot_pose", Pose2D, pose_callback)
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

    fig = plt.figure("Pose + Obstacle Plot")
    ani = animation.FuncAnimation(fig, animate, interval=500)

    plt.tight_layout()
    plt.show()
    rospy.spin()

if __name__ == '__main__':
    main()