#!/usr/bin/env python3
import rospy
import sys
import logging
import time
from std_msgs.msg import Float32, String
import geometry_msgs.msg as geometry_msgs
import agiros_msgs.msg as agiros_msgs
import numpy as np
from geometry_msgs.msg import TwistStamped, Twist, Vector3
import threading
import numpy.linalg as la
import pickle
import time


class Deploy:
    def __init__(self):
        self.target_pose = None
        self.current_pose = None
        self.pub = None
        self.stop_event = threading.Event() # Create a global event that can be used to signal the loop to stop
        self.position_history = []
        

        # Control Parameters:
        self.K_p = 1.2
        self.K_i = 0.6
        self.K_d = 0.0
        self.PID_integral_max = 10
        self.PID_integral_min = -10
        self.ctrl_dt = 0.05

        # Debug use:
        self.debug_callback_time = None
        self.debug_vel_publish_time = None
        self.time_record = []
        self.time_start = time.time()

        try:
            self.ctrl_thread = threading.Thread(target=self.control_scheduler, args=())
            self.ctrl_thread.start()
            self.main()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROSInterruptException caught")
        finally:
            self.stop_threads()
            rospy.loginfo("EXITING Main")
            with open("Fruits.pkl", "wb") as filehandler:
                pickle.dump(self.position_history, filehandler)
            rospy.loginfo("Finished Script")

            runtimes = np.array(self.time_record)
            print("datapoints num: ", runtimes.shape)
            print("max: ",np.max(runtimes))
            print("min: ",np.min(runtimes))
            print("std: ",np.std(runtimes))
            print("mean: ",np.mean(runtimes))



    def main(self):
        rospy.init_node('deploy_node', anonymous=True)

        self.configure_logging()
        rospy.loginfo("Experiment: {}".format(self.time_start))

        sub_startpoint = rospy.Subscriber("/kingfisher/agiros_pilot/state", agiros_msgs.QuadState, self.callback_state)

        # Flight with vision
        # sub_aruco = rospy.Subscriber("/leader_waypoint", geometry_msgs.Point, self.callback_target)
        # Debugging with fake_cam publisher
        sub_aruco = rospy.Subscriber("/fake_waypoint", geometry_msgs.Point, self.callback_target)

        quad_namespace = "kingfisher"

        # self.pub_pos = rospy.Publisher(quad_namespace+"/agiros_pilot/go_to_pose", 
        #                     geometry_msgs.PoseStamped, 
        #                     queue_size=1)

        self.pub = rospy.Publisher(quad_namespace+"/agiros_pilot/velocity_command", 
                        geometry_msgs.TwistStamped, 
                        queue_size=1)

        rate = rospy.Rate(40)  # 10 Hz
        
        print("Ready for Tracking ......")
        while not rospy.is_shutdown():
            # Process callbacks and wait for messages
            rospy.spin()
            # # Sleep to control loop rate
            rate.sleep()
        
    def configure_logging(self):
        # Create a logger
        logger = logging.getLogger('rosout')
        logger.setLevel(logging.INFO)

        # Create a file handler
        log_file = 'logfile_deploy2.log'  # Update this path
        fh = logging.FileHandler(log_file)
        fh.setLevel(logging.INFO)

        # Create a formatter and set it for the handler
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        fh.setFormatter(formatter)

        # Add the file handler to the logger
        logger.addHandler(fh)

    def callback_state(self, data):
        self.current_pose = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        
        # rospy.loginfo("Recieving State: {},{},{}".format(global_x,global_y,global_z))
    
    def callback_target(self, data):
        self.debug_callback_time = time.time()
        self.target_pose = np.array([data.x+self.current_pose[0], data.y+self.current_pose[1], data.z+self.current_pose[2]])


    def control_scheduler(self):
        first = True
        while not self.stop_event.is_set():
            # check for new target points
            if self.target_pose is not None:
                active_target_pose = np.copy(self.target_pose)

                integral_error = np.array([0.0, 0.0, 0.0])
                previous_error = np.array([0.0, 0.0, 0.0])

                rospy.loginfo("Starting velPID ctrl, current state: {}, target pos: {}".format(self.current_pose, active_target_pose))
                debug_switch = True

                while (not self.stop_event.is_set() 
                    and self.current_pose is not None 
                    and np.array_equal(active_target_pose, self.target_pose)
                    and not self._reached_target_position(self.current_pose, active_target_pose)):
                    
                    #MATBE CHANGE
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
                    if debug_switch:
                        self.debug_vel_publish_time = time.time()
                        self.time_record.append(self.debug_vel_publish_time-self.debug_callback_time)
                        debug_switch=False
                    rospy.loginfo("Publishing VelY to Ctrl: {}, Current state :{}".format(velocity_command[1], self.current_pose))

                    # Update previous error
                    previous_error = position_error
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

    def stop_threads(self):
        self.stop_event.set()
        self.ctrl_thread.join()

if __name__ == '__main__':
    dep = Deploy()
    dep.main()