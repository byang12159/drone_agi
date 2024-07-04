#!/usr/bin/env python3
import rospy
import sys
import logging
import time
from std_msgs.msg import Float32, String, Bool
import geometry_msgs.msg as geometry_msgs
import agiros_msgs.msg as agiros_msgs
import numpy as np
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Point
import threading
import numpy.linalg as la
import pickle
import time


target_pose = None
new_message_received = False
current_pose = None

class PID_Controller:
    def __init__(self): 
        global target_pose, current_pose
      
        # Control Parameters:
        self.K_p = 1.2
        self.K_i = 0.6
        self.K_d = 0.0
        self.PID_integral_max = 10
        self.PID_integral_min = -10
        self.ctrl_dt = 0.05

        self.position_history = []

        rospy.init_node('controller_node', anonymous=True)
        
        quad_namespace = "kingfisher"


        # self.pub_pos = rospy.Publisher(quad_namespace+"/agiros_pilot/go_to_pose",
        #                     geometry_msgs.PoseStamped,
        #                     queue_size=1)


        self.pub = rospy.Publisher(quad_namespace+"/agiros_pilot/velocity_command",
                        geometry_msgs.TwistStamped,
                        queue_size=1)
        
        print("watiign for PF")
        rospy.wait_for_message('/PF_output', Point)

        print("recieved PF")
        rospy.Subscriber('/PF_output',Point, self.callback,queue_size=1)
        self.sub_startpoint = rospy.Subscriber("/kingfisher/agiros_pilot/state", agiros_msgs.QuadState, self.callback_state, queue_size=1)

        while target_pose is None and current_pose is None:
            rospy.sleep(0.1)  # Let the state_current be updated


    def callback(self, data):
        global new_message_received, target_pose
        new_message_received = True
        target_pose = np.array([data.x, data.y, data.z])
    
    def callback_state(self, data):
       global current_pose
       current_pose = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    

    def main(self):
        global new_message_received, target_pose, current_pose
        print("Staring Main")
        print("TARGET",target_pose)
        while not rospy.is_shutdown():
 
            new_message_received = False

            # Inner loop runs until a new message is received
            while True:
                active_target_pose = np.copy(target_pose)


                integral_error = np.array([0.0, 0.0, 0.0])
                previous_error = np.array([0.0, 0.0, 0.0])


                rospy.loginfo("Starting velPID ctrl, current state: {}, target pos: {}".format(current_pose, active_target_pose))

                while not self._reached_target_position(current_pose, active_target_pose):
    
                    target_pose = np.copy(target_pose)

                    self.position_history.append(current_pose)


                    # Position Error
                    position_error = target_pose - current_pose
                    # Integral Error
                    integral_error += position_error * self.ctrl_dt
                    integral_error[0] = max(min(integral_error[0], self.PID_integral_max), self.PID_integral_min)  # Clamping
                    integral_error[1] = max(min(integral_error[1], self.PID_integral_max), self.PID_integral_min)  # Clamping
                    integral_error[2] = max(min(integral_error[2], self.PID_integral_max), self.PID_integral_min)  # Clamping

                    # Derivative Error
                    derivative_error = (position_error - previous_error) / self.ctrl_dt


                    # Compute PID velocity command
                    velocity_command = self.K_p * position_error + self.K_i * integral_error + self.K_d * derivative_error
                    
                    # print("cmd", velocity_command)
                    velocity_command = self._limitVelocity(velocity_command)


                    # Send velocity commands to the quadcopter
                    print("cmd", velocity_command)
                    vel_cmd = geometry_msgs.TwistStamped()
                    vel_cmd.twist.linear = Vector3(0.0, velocity_command[1], 0.0)
                    vel_cmd.twist.angular = Vector3(0.0, 0.0, 0.0)
                    self.pub.publish(vel_cmd)
                    rospy.loginfo("Publishing VelY to Ctrl: {}, Current state :{}".format(velocity_command[1], current_pose))

                    # Update previous error
                    previous_error = position_error

                    if new_message_received or np.array_equal(active_target_pose, target_pose)==False:
                        break

                    rospy.sleep(self.ctrl_dt)  # Sleep for a while to simulate work


                rospy.loginfo("Exiting velPID loop")

  
    def _reached_target_position(self, current_position, target_position):
        position_tolerance = 0.01
        return la.norm(current_position - target_position) < position_tolerance


    def _limitVelocity(self, velocity):
        # Only check y now
        max_velocity = 4.0
        velocity[1] = np.clip(velocity[1], -max_velocity, max_velocity)
        
        return velocity


if __name__ == '__main__':
   ctrl = PID_Controller()
   ctrl.main()

