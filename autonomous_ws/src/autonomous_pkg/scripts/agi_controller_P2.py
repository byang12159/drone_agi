#!/usr/bin/env python3
import rospy
import sys
import logging
import time
from std_msgs.msg import Float32, String, Bool,Float64MultiArray
import geometry_msgs.msg as geometry_msgs
import agiros_msgs.msg as agiros_msgs
import numpy as np
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Point, Pose
import numpy.linalg as la
import pickle
import time
import signal
import numpy.linalg as la


target_pose = None
target_pose_p = None
new_message_received = False
current_pose = None
prediction_on = False
prediction_count = 40
data_storage = []
state_est = None
vicon_pose = None
accumulated_backoff = 0
current_mode = 1
mode_count = 0
# did_prediction = False


class PID_Controller:
    def __init__(self): 
        global target_pose, current_pose, vicon_pose
      
        # Control Parameters:
        self.K_p = 0.85
        self.K_i = 0.6
        self.K_d = 0.0

        # self.K_p = 0.72
        # self.K_i = 0.918
        # self.K_d = 0.23

        self.PID_integral_max = 10
        self.PID_integral_min = -10

        # self.position_history = []

        rospy.init_node('controller_node', anonymous=True)
        
        self.ctrl_dt = 0.01
        self.rate = rospy.Rate(100)  # Hz

        self.pub = rospy.Publisher("/kingfisher/agiros_pilot/velocity_command",
                        geometry_msgs.TwistStamped,
                        queue_size=1)
        
        self.pub_log = rospy.Publisher('/log_messages_ctrl', Float64MultiArray, queue_size=10)
        self.log_data = Float64MultiArray()
        self.log_data.data = [0, ] * (2 + 3 + 3 + 3 + 1)


        # rospy.Subscriber('/fake_waypoint',Point, self.callback,queue_size=1)      
        # rospy.Subscriber('/leader_waypoint',Point, self.callback,queue_size=1)

        sub_startpoint = rospy.Subscriber("/kingfisher/agiros_pilot/state", agiros_msgs.QuadState, self.callback_state, queue_size=1)
        sub_detection_bool = rospy.Subscriber('/aruco_detection',Bool, self.callback_detection_bool,queue_size=1)
        sub_PF = rospy.Subscriber('/prediction_output',Pose, self.callback_Prediction,queue_size=1)
        sub_vicon = rospy.Subscriber("/vicon_estimate", Float64MultiArray, self.callback_vicon, queue_size=1)

        signal.signal(signal.SIGINT, self.signal_handler)

        while target_pose is None or vicon_pose is None:
            rospy.sleep(0.1) 

        self.initial_vicon_state = vicon_pose

    def signal_handler(self, sig, frame):
        rospy.loginfo("Ctrl+C pressed. Exiting...")
        rospy.signal_shutdown("Ctrl+C pressed")
        sys.exit(0)

    def callback(self, data):
        global new_message_received, target_pose, current_pose,accumulated_backoff
        new_message_received = True
    
        target_pose = np.array([data.x+current_pose[0]-1.5, data.y+current_pose[1], data.z+current_pose[2]])
        
        print("Spotted Marker",target_pose)
    
    def callback_state(self, data):
       global current_pose
       current_pose = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    
    def callback_vicon(self, data):
        global vicon_pose
        vicon_pose = np.array([data.data[0], data.data[1], data.data[2]])

    def callback_detection_bool(self, data):
        global prediction_on,prediction_count, data_storage
        if data.data:
            prediction_on = False   
            prediction_count = 40
        else:
            prediction_on = True
            prediction_count+=1
        # data_storage.append([prediction_on,prediction_count,time.time()])
        
            
    def callback_Prediction(self, data):
        global target_pose, current_pose, new_message_received, prediction_count, accumulated_backoff, current_mode, mode_count
        new_message_received = True
        # backoff = 0.01*(prediction_count)*data.x
        # if accumulated_backoff<-1.5:
        #     backoff=0
        target_pose = np.array([data.position.x+current_pose[0], data.position.y+current_pose[1], data.position.z+current_pose[2]])
        current_mode = int(data.orientation.x)

        if current_mode == 4:
            print("Prediction Mode: ",current_mode, " Target: ",target_pose)

    def PI_loop(self,):
        global new_message_received, target_pose, current_pose, prediction_on, target_pose_p, accumulated_backoff

        active_target_pose = np.copy(target_pose)

        integral_error = np.array([0.0, 0.0, 0.0])
        previous_error = np.array([0.0, 0.0, 0.0])

        start_x = None
        if prediction_on:
            start_x = current_pose[0]


        # rospy.loginfo("Starting velPID ctrl, current state: {}, target pos: {}".format(current_pose, active_target_pose))
        while not rospy.is_shutdown() and not self._reached_target_position(current_pose, active_target_pose):
            starttime = time.time()

            # target_pose = np.copy(target_pose)

            # self.position_history.append(current_pose)

            # Position Error
            position_error = active_target_pose - current_pose
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

            velocity_command = self._limitPos(velocity_command)
     
            vel_cmd = geometry_msgs.TwistStamped()
            vel_cmd.twist.linear = Vector3(velocity_command[0], velocity_command[1], velocity_command[2])
            vel_cmd.twist.angular = Vector3(0.0, 0.0, 0.0)
            self.pub.publish(vel_cmd)
            # rospy.loginfo("Publishing VelY to Ctrl: {}, Current state :{}, timestamp: {}".format(velocity_command[1], current_pose, time.time()))

            # Update previous error
            previous_error = position_error
            # print("runtime: ",time.time()-starttime)

            if new_message_received:
                break

            # if velocity_command[0] >= 1 and current_mode ==4:
            #     rospy.signal_shutdown("Manual shutdown")

            self.rate.sleep()  
   
    def main(self):
        global new_message_received,target_pose, current_pose, prediction_on, mode_count

        while not rospy.is_shutdown():
            #Add prediciotn siwth here
 
            new_message_received = False

            # Inner loop runs until a new message is received
            self.PI_loop()

            # ROS Logging 
            self.log_data.data[0] = int(prediction_on)
            self.log_data.data[1] = int(new_message_received)
            self.log_data.data[2] = current_pose[0]
            self.log_data.data[3] = current_pose[1]
            self.log_data.data[4] = current_pose[2]
            self.log_data.data[5] = target_pose[0]
            self.log_data.data[6] = target_pose[1]
            self.log_data.data[7] = target_pose[2]
            self.log_data.data[8] = vicon_pose[0]
            self.log_data.data[9] = vicon_pose[1]
            self.log_data.data[10] = vicon_pose[2]
            now = rospy.get_rostime()
            now = now.to_sec()
            self.log_data.data[-1] = now
            self.pub_log.publish(self.log_data)

            print("Exiting Vel-PID loop")

  
    def _reached_target_position(self, current_position, target_position):
        position_tolerance = 0.005
        return la.norm(current_position - target_position) < position_tolerance

    def _limitVelocity(self, velocity):
        global current_mode, mode_count
        # current_mode = 1
        if current_mode < 4:
            max_velocity_x = 1.0
            mode_count = 0
        else:
            # exp_vel = np.exp(0.01 * mode_count) - 0.8
            exp_vel = 1/100*mode_count+0.2
            print("EXP", exp_vel)
            max_velocity_x = min(1.0, exp_vel)
            # max_velocity_x = 0.2
            mode_count += 1

        
        # exp_vel = np.exp(0.01 * mode_count) - 0.8
        # max_velocity_x = min(1.0, exp_vel)    
        # # max_velocity_x = 0.2
        max_velocity = 1.0

        velocity[0] = np.clip(velocity[0], -max_velocity_x, max_velocity_x)
        velocity[1] = np.clip(velocity[1], -max_velocity, max_velocity)
        velocity[2] = np.clip(velocity[2], -max_velocity, max_velocity)

        # if current_mode == 4:          
        print("velocity: ", velocity)

        return velocity
    
    def _limitPos(self, velocity):
        global vicon_pose

        max_y = 3.2
        min_y = -2.1
        max_x = 3.1
        min_x = -2.3
        max_z = 3.5
        
        if vicon_pose[0]>=max_x or vicon_pose[0]<=min_x:
            print("LIMIT REACHED X: {}".format(vicon_pose[0]))
            velocity[0] =0.0
        if vicon_pose[1]>=max_y or vicon_pose[1]<=min_y:
            print("LIMIT REACHED Y: {}".format(vicon_pose[1]))
            velocity[1] = 0.0
        if vicon_pose[2]>=max_z:
            print("LIMIT REACHED Z: {}".format(vicon_pose[2]))
            velocity[2] = 0.0
        
        return velocity



if __name__ == '__main__':
    ctrl = PID_Controller()
    try:
        ctrl.main()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException caught")

    
    # save_file = "data_aruco.pkl"

    # with open(save_file ,'wb') as file:
    #     pickle.dump(data_storage,file)

    # print("finished controller log, saved to {}".format(save_file))

