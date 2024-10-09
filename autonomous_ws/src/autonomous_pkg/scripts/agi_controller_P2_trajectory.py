#!/usr/bin/env python3
import rospy
import sys
import logging
import time
from std_msgs.msg import Float32, String, Bool,Float64MultiArray
import geometry_msgs.msg as geometry_msgs
import agiros_msgs.msg as agiros_msgs
import numpy as np
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Point, Pose, PointStamped, PoseArray, Quaternion
import numpy.linalg as la
import pickle
import time
import signal
import numpy.linalg as la
from pyquaternion import Quaternion

target_pose = None
target_pose_p = None
new_message_received = False
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
        self.K_p = 1.2
        self.K_i = 0.9
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
        self.log_data.data = [0, ] * (2 + 3 + 3 + 3 + 2 +1)


        # rospy.Subscriber('/fake_waypoint',Point, self.callback_fake,queue_size=1)      
        rospy.Subscriber('/fake_waypoint_list', PoseArray, self.callback_fake, queue_size=1)

        self.waypoints_list = np.array([])
        self.accumulated_dist_list = np.array([])
        self.current_waypoint_id = 0
        self.active_target_pose = None
        # rospy.Subscriber('/leader_waypoint',PointStamped, self.callback,queue_size=1)

        self.current_pose = None 

        sub_startpoint = rospy.Subscriber("/kingfisher/agiros_pilot/state", agiros_msgs.QuadState, self.callback_state, queue_size=1)
        sub_detection_bool = rospy.Subscriber('/aruco_detection',Bool, self.callback_detection_bool,queue_size=1)
        sub_PF = rospy.Subscriber('/prediction_output',Pose, self.callback_Prediction,queue_size=1)
        sub_vicon = rospy.Subscriber("/vicon_estimate", Float64MultiArray, self.callback_vicon, queue_size=1)

        signal.signal(signal.SIGINT, self.signal_handler)

        while self.waypoints_list.size<=0 or self.accumulated_dist_list.size<=0 or vicon_pose is None or self.current_pose is None:
            print(self.waypoints_list.size, self.accumulated_dist_list.size, vicon_pose, self.current_pose)
            rospy.sleep(0.1) 

        print("started")

        self.initial_vicon_state = vicon_pose

    def signal_handler(self, sig, frame):
        rospy.loginfo("Ctrl+C pressed. Exiting...")
        rospy.signal_shutdown("Ctrl+C pressed")
        sys.exit(0)

    def callback(self, data:PointStamped):
        global new_message_received, target_pose,accumulated_backoff
        new_message_received = True
    
        target_pose = np.array([data.point.x+self.current_pose[0]-1.5, data.point.y+self.current_pose[1], data.point.z+self.current_pose[2]])
        
        print("Spotted Marker",target_pose)

    def callback_fake(self, data:PoseArray):
        # global new_message_received, target_pose, current_pose,accumulated_backoff
        # new_message_received = True
    
        # target_pose = np.array([data.x, data.y, data.z])
        
        # print("Spotted Marker",target_pose)
        if self.waypoints_list.size>0:
            print("rejecting repeated waypoints list")
            return
        print("message received")
        posearray = data.poses
        print("Length: {}".format(len(posearray))) 
        waypoints_list = []
        accumulated_dist_list = []
        for i in range(len(posearray)):
            pose = posearray[i]
            pos = pose.position 
            ori = pose.orientation

            x,y,z = pos.x, pos.y, pos.z 
            qx, qy, qz, qw = ori.x, ori.y, ori.z, ori.w 
            # print("qx, qy, qz, qw",qx, qy, qz, qw)

            # yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            q = Quaternion(qw, qx, qy, qz)
            yaw = q.yaw_pitch_roll[0]

            waypoints_list.append([x,y,z,yaw])

            if i==0:
                accumulated_dist_list.append(0)
            else:
                prev_x, prev_y, prev_z = posearray[i-1].position.x,posearray[i-1].position.y,posearray[i-1].position.z
                accumulated_dist_list.append(
                    accumulated_dist_list[-1]
                    + np.linalg.norm([x-prev_x, y-prev_y, z-prev_z]))

        waypoints_list = np.array(waypoints_list)
        dist = np.linalg.norm(self.current_pose-waypoints_list[0,:])
        if dist>0.5:
            print("Starting point far away from current pose, Reject")
            return 
        self.waypoints_list = waypoints_list
        self.current_waypoint_id = 0
        self.accumulated_dist_list = np.array(accumulated_dist_list)
        self.active_target_pose = self.waypoints_list[0,:]
        np.savez('waypoints.npz', waypoints_list = self.waypoints_list, accumulated_dist_list = self.accumulated_dist_list)
    
    def callback_state(self, data):
    #    global current_pose
        qx, qy, qz, qw = data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w
        # yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        q = Quaternion(qw, qx, qy, qz)
        yaw = q.yaw_pitch_roll[0]

        # print("Current pose: {}, {}, {}, {}".format(data.pose.position.x, data.pose.position.y, data.pose.position.z, yaw))
        self.current_pose = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z, yaw])
    
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
        global target_pose, new_message_received, prediction_count, accumulated_backoff, current_mode, mode_count
        new_message_received = True
        # backoff = 0.01*(prediction_count)*data.x
        # if accumulated_backoff<-1.5:
        #     backoff=0
        target_pose = np.array([data.position.x+self.current_pose[0], data.position.y+self.current_pose[1], data.position.z+self.current_pose[2]])
        current_mode = int(data.orientation.x)

        if current_mode == 4:
            print("Prediction Mode: ",current_mode, " Target: ",target_pose)

    def get_front_back_offset(self, front_back_dist=2.0):
        waypoint_dist = self.accumulated_dist_list[self.current_waypoint_id:]-self.accumulated_dist_list[self.current_waypoint_id]
        idx = np.argmin(np.abs(waypoint_dist - front_back_dist))
        return idx


    def compute_current_waypoint(self):
        current_pos = self.current_pose[:3]
        front_back_offset = self.get_front_back_offset()
        dist = np.linalg.norm(self.waypoints_list[max(self.current_waypoint_id-front_back_offset,0):self.current_waypoint_id+front_back_offset, :3]-current_pos, axis=1)
        # indices = np.where(dist <= 0.5)[0]
        # if indices.size > 0:
        #     idx = indices[0]+self.current_waypoint_id
        #     idx = np.argmin(dist[indices[0]:indices[0]+50])+indices[0]+self.current_waypoint_id
        # else:
        idx = np.argmin(dist)+max(self.current_waypoint_id-front_back_offset,0)
        print(self.current_waypoint_id, dist[idx-self.current_waypoint_id])
        self.current_waypoint_id = idx
        return idx
    
    def compute_active_target_pose(self):
        current_pos = self.current_pose[:3]
        active_target_pos = self.active_target_pose[:3]
        dist = np.linalg.norm(current_pos-active_target_pos)
        if dist > 0.1:
            return self.active_target_pose, self.current_waypoint_id
        else:
            pass 
            

    def normalize_angle(self, angle):
        """Normalize angle to be within [-pi, pi]."""
        angle = angle%(np.pi*2.0)
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def compute_look_ahead(self, target_idx, lookahead_dist = 0.1):
        waypoint_dist = self.accumulated_dist_list[target_idx:]-self.accumulated_dist_list[target_idx]
        idx = np.argmin(np.abs(waypoint_dist - lookahead_dist))
        return idx

    def PI_loop(self,):
        global new_message_received, target_pose, prediction_on, target_pose_p, accumulated_backoff

        # active_target_pose = np.copy(target_pose)

        integral_error = np.array([0.0, 0.0, 0.0])
        previous_error = np.array([0.0, 0.0, 0.0])

        start_x = None
        if prediction_on:
            start_x = self.current_pose[0]

        prev_yaw_error = None
        # rospy.loginfo("Starting velPID ctrl, current state: {}, target pos: {}".format(self.current_pose, active_target_pose))
        while not rospy.is_shutdown():
            print("pid loop")
            starttime = time.time()
            if self.waypoints_list.size<=0:
                break 


            # target_pose = np.copy(target_pose)

            # self.position_history.append(self.current_pose)
            target_idx = self.compute_current_waypoint()
            look_ahead = self.compute_look_ahead(target_idx, lookahead_dist=0.2)
            print("Loop running, waypoint idx: {}".format(target_idx+look_ahead))
            print("Current Pose: {}".format(self.current_pose))
            if target_idx+look_ahead>= self.waypoints_list.shape[0]:
                active_target_pose = self.waypoints_list[-1,:] # TODO: Tweak this                 
            else:
                active_target_pose = self.waypoints_list[target_idx+look_ahead,:] # TODO: Tweak this 
            print("Current waypoint: {}".format(active_target_pose))
            # Position Error
            position_error = active_target_pose[:3] - self.current_pose[:3]
            yaw_error = active_target_pose[3] - self.current_pose[3]
            yaw_error = self.normalize_angle(yaw_error)
            if prev_yaw_error is None:
                prev_yaw_error = yaw_error 
            elif abs(yaw_error-prev_yaw_error)>np.pi/2:
                print("Huge yaw eror {}, {}, {}".format(yaw_error,prev_yaw_error,abs(yaw_error-prev_yaw_error)))
                # break
            # Integral Error
            integral_error += position_error * self.ctrl_dt
            integral_error[0] = max(min(integral_error[0], self.PID_integral_max), self.PID_integral_min)  # Clamping
            integral_error[1] = max(min(integral_error[1], self.PID_integral_max), self.PID_integral_min)  # Clamping
            integral_error[2] = max(min(integral_error[2], self.PID_integral_max), self.PID_integral_min)  # Clamping

            # Derivative Error
            derivative_error = (position_error - previous_error) / self.ctrl_dt

            # Compute PID velocity command
            velocity_command = self.K_p * position_error + self.K_i * integral_error + self.K_d * derivative_error
            
            # Angular velocit coommand 
            velocity_angular = 2 * yaw_error
            print(active_target_pose[3], self.current_pose[3], yaw_error, velocity_angular)

            # print("cmd", velocity_command)
            velocity_command = self._limitVelocity(velocity_command)

            velocity_command = self._limitPos(velocity_command)
     
            vel_cmd = geometry_msgs.TwistStamped()
            vel_cmd.twist.linear = Vector3(velocity_command[0], velocity_command[1], velocity_command[2])
            vel_cmd.twist.angular = Vector3(0.0, 0.0, velocity_angular)
            self.pub.publish(vel_cmd)
            # rospy.loginfo("Publishing VelY to Ctrl: {}, Current state :{}, timestamp: {}".format(velocity_command[1], self.current_pose, time.time()))

            # Update previous error
            previous_error = position_error
            # print("runtime: ",time.time()-starttime)

            # if new_message_received:
            #     break

            # if velocity_command[0] >= 1 and current_mode ==4:
            #     rospy.signal_shutdown("Manual shutdown")

            # ROS Logging 
            self.log_data.data[0] = int(prediction_on)
            self.log_data.data[1] = int(new_message_received)
            self.log_data.data[2] = self.current_pose[0]
            self.log_data.data[3] = self.current_pose[1]
            self.log_data.data[4] = self.current_pose[2]
            self.log_data.data[5] = active_target_pose[0]
            self.log_data.data[6] = active_target_pose[1]
            self.log_data.data[7] = active_target_pose[2]
            self.log_data.data[8] = vicon_pose[0]
            self.log_data.data[9] = vicon_pose[1]
            self.log_data.data[10] = vicon_pose[2]
            self.log_data.data[11] = target_idx 
            self.log_data.data[12] = look_ahead 
            now = rospy.get_rostime()
            now = now.to_sec()
            self.log_data.data[-1] = now
            self.pub_log.publish(self.log_data)
            self.rate.sleep()  
   
    def main(self):
        global new_message_received,target_pose, prediction_on, mode_count

        while not rospy.is_shutdown():
            #Add prediciotn siwth here

 
            new_message_received = False

            # Inner loop runs until a new message is received
            self.PI_loop()

            # ROS Logging 
            self.log_data.data[0] = int(prediction_on)
            self.log_data.data[1] = int(new_message_received)
            self.log_data.data[2] = self.current_pose[0]
            self.log_data.data[3] = self.current_pose[1]
            self.log_data.data[4] = self.current_pose[2]
            self.log_data.data[5] = 0
            self.log_data.data[6] = 0
            self.log_data.data[7] = 0
            self.log_data.data[8] = vicon_pose[0]
            self.log_data.data[9] = vicon_pose[1]
            self.log_data.data[10] = vicon_pose[2]
            now = rospy.get_rostime()
            now = now.to_sec()
            self.log_data.data[-1] = now
            self.pub_log.publish(self.log_data)

            print("Exiting Vel-PID loop")
            break

  
    def _reached_target_position(self, current_position, target_position):
        position_tolerance = 0.005
        return la.norm(current_position - target_position) < position_tolerance

    def _limitVelocity(self, velocity):
        global current_mode, mode_count
        # current_mode = 1
        if current_mode < 4:
            max_velocity_l = 1.0
            mode_count = 0
        else:
            # exp_vel = np.exp(0.01 * mode_count) - 0.8
            exp_vel = 1/100*mode_count+0.05 
            print("EXP", exp_vel)
            max_velocity_l = min(1.0, exp_vel)
            # max_velocity_l = 0.2
            mode_count += 1

        
        # exp_vel = np.exp(0.01 * mode_count) - 0.8
        # max_velocity_l = min(1.0, exp_vel)    
        # # max_velocity_l = 0.2
        max_velocity = 1.0

        velocity[0] = np.clip(velocity[0], -max_velocity_l, max_velocity_l)
        velocity[1] = np.clip(velocity[1], -max_velocity, max_velocity)
        velocity[2] = np.clip(velocity[2], -max_velocity, max_velocity)

        # if current_mode == 4:          
        print("velocity: ", velocity)

        return velocity
    
    def _limitPos(self, velocity):
        global vicon_pose

        # max_x = 3.0
        # min_x = -2.6
        # max_y = 1.8
        # min_y = -3.6
        # max_z = 3.5
        # (4.2, 3.9)
        # (-2.5, 3.9)
        # (-2.5,-3.5)
        # (4.2, -3.5)
        # max_y = 3.6
        # min_y = -3.2
        # max_x = 3.9
        # min_x = -2.2
        # max_z = 2.2
        # min_z = 0.3

        max_y = 3.2
        min_y = -2.4
        max_x = 3.1
        min_x = -2.0
        max_z = 3.0
        min_z = 0.1

        if vicon_pose[0]>=max_x or vicon_pose[0]<=min_x:
            print("LIMIT REACHED X: {}".format(vicon_pose[0]))
            velocity[0] =0.0
        if vicon_pose[1]>=max_y or vicon_pose[1]<=min_y:
            print("LIMIT REACHED Y: {}".format(vicon_pose[1]))
            velocity[1] = 0.0
        if vicon_pose[2]>=max_z or vicon_pose[2]<=min_z:
            print("LIMIT REACHED Z: {}".format(vicon_pose[2]))
            velocity[2] = 0.0

        if vicon_pose[2]>=2.7:
            print(f"DANGEROUS HEIGHT Z: {vicon_pose[2]}; STOPPING")
            velocity[0] = 0
            velocity[1] = 0
            velocity[2] = 0
        
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

