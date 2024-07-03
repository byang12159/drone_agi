#!/usr/bin/env python3
import rospy
import sys
import logging
import time
from std_msgs.msg import Float32, String, Bool
import geometry_msgs.msg as geometry_msgs
import agiros_msgs.msg as agiros_msgs
import numpy as np
from geometry_msgs.msg import TwistStamped, Twist, Vector3
import threading
import numpy.linalg as la
import pickle
import time


class PID_Controller:
    def __init__(self): 

      
        # Control Parameters:
        self.K_p = 1.2
        self.K_i = 0.6
        self.K_d = 0.0
        self.PID_integral_max = 10
        self.PID_integral_min = -10
        self.ctrl_dt = 0.05


        rospy.init_node('controller_node', anonymous=True)
        rospy.Subscriber('/PF_output',self.callback,queue_size=1)

        self.target_global = None
        self.new_message_received = False

    def callback(self, data):
        self.new_message_received = True
        self.target_global = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    

    def main(self):
       
        while not rospy.is_shutdown() and self.target_global != None:
 
            self.new_message_received = False

            # Inner loop runs until a new message is received
            while True:
                active_target_pose = np.copy(self.target_pose)


                integral_error = np.array([0.0, 0.0, 0.0])
                previous_error = np.array([0.0, 0.0, 0.0])


                rospy.loginfo("Starting velPID ctrl, current state: {}, target pos: {}".format(self.current_pose, active_target_pose))

                while not self._reached_target_position(self.current_pose, active_target_pose):
                    
    
                    target_pose = np.copy(self.target_pose)

                    self.position_history.append(self.current_pose)


                    # Position Error
                    position_error = target_pose - self.current_pose
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
                    rospy.loginfo("Publishing VelY to Ctrl: {}, Current state :{}".format(velocity_command[1], self.current_pose))

                    # Update previous error
                    previous_error = position_error

                    if self.new_message_received or np.array_equal(active_target_pose, self.target_pose)==False:
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
