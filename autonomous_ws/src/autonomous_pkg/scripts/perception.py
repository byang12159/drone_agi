#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
import sys
import logging
import time
from std_msgs.msg import Bool,Float64MultiArray
import geometry_msgs.msg as geometry_msgs
import agiros_msgs.msg as agiros_msgs
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Point, Pose, PointStamped, PoseArray, Quaternion
import signal
import rrt_star_3d

camera_mtx = np.array([[546.84164912 ,  0. ,     349.18316327],
                [  0.   ,      547.57957461 ,215.54486004],
                [  0.    ,       0.     ,      1.        ]])

class Perception_Module:
    def __init__(self): 
        self.current_gate_pose = None
        self.gate_detec = False
        self.current_drone_pose = None
        self.vicon_pose = None

        rospy.init_node('perception_node', anonymous=True)
        
        self.ctrl_dt = 0.01
        self.rate = rospy.Rate(100)  # Hz

        # self.pub = rospy.Publisher("/kingfisher/agiros_pilot/velocity_command",
        #                 geometry_msgs.TwistStamped,
        #                 queue_size=1)

        print("trying to subscribe")
        # sub_state = rospy.Subscriber("/kingfisher/agiros_pilot/state", agiros_msgs.QuadState, self.callback_state, queue_size=1)
        sub_detection_bool = rospy.Subscriber('/gate_detection', Bool, self.callback_detection_bool,queue_size=1)
        sub_gate_pose = rospy.Subscriber('/gate_pos', PointStamped, self.point_callback,queue_size=1)
        sub_vicon = rospy.Subscriber("/vicon_estimate", Float64MultiArray, self.callback_vicon, queue_size=1)

        signal.signal(signal.SIGINT, self.signal_handler)
        
        #  or self.current_drone_pose is None
        while self.vicon_pose is None:
            rospy.sleep(0.1) 

        print("started")

        self.initial_vicon_state = self.vicon_pose

    # Callback function for the PointStamped topic
    def point_callback(self, msg):
        # rospy.loginfo("Received PointStamped message:")
        # rospy.loginfo("  Header:")
        # rospy.loginfo("    Frame ID: %s", msg.header.frame_id)
        # rospy.loginfo("    Timestamp: %s", msg.header.stamp)
        # rospy.loginfo("  Point:")
        # rospy.loginfo("    x: %f", msg.point.x)
        # rospy.loginfo("    y: %f", msg.point.y)
        # rospy.loginfo("    z: %f", msg.point.z)
        rospy.loginfo(f"x: {msg.point.x}, y: {msg.point.y}, z:{msg.point.z}")
        self.current_gate_pose = np.array([msg.point.x, msg.point.y, msg.point.z])

    def callback_detection_bool(self, msg):
        # print('here')
        self.gate_detec = msg.data

    def callback_state(self, data):
        # print('there')
        qx, qy, qz, qw = data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w
        q = Quaternion(qw, qx, qy, qz)
        yaw, pitch, roll = q.yaw_pitch_roll

        self.current_drone_pose = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z, roll, pitch, yaw])

    def callback_vicon(self, data):
        self.vicon_pose = np.array([data.data[0], data.data[1], data.data[2]])

    def signal_handler(self, sig, frame):
        rospy.loginfo("Ctrl+C pressed. Exiting...")
        rospy.signal_shutdown("Ctrl+C pressed")
        sys.exit(0)

    def main(self):
        print(f'gate detec: {self.gate_detec}')
        print(f'current_gate_pose: {self.current_gate_pose}')
        print(f'current_drone_pose: {self.current_drone_pose}')
        #  and self.current_drone_pose:
        if self.gate_detec and self.current_gate_pose is not None:
            gate_x, gate_y, gate_z = self.current_gate_pose
            # drone_x, drone_y, drone_z = self.current_drone_pose
            K_inv = np.linalg.inv(camera_mtx)

            desired_waypoint = K_inv @ np.array([gate_x, gate_y, 1]).T
            print(desired_waypoint)
            # desired_waypoint = []
            # return rrt_star_3d.cam_plan(self.current_drone_pose, desired_waypoint)
        else:
            print(f"Failed to detect. \nGate detection: {self.gate_detec}\nGate position: {self.current_gate_pose}\nDrone position: {self.current_drone_pose}\n")
            return None

if __name__ == '__main__':
    perc = Perception_Module()
    try:
        perc.main()
    except rospy.ROSInterruptException:
        pass
