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


class Deploy:
    def __init__(self):
        self.target_poses = []
        self.current_pose = None
        self.pub = None
        self.stop_event = threading.Event() # Create a global event that can be used to signal the loop to stop
        self.position_history = []

        self.target_queue = 10

        # Control Parameters:
        self.K_p = 1.2
        self.K_i = 0.6
        self.K_d = 0.0
        self.PID_integral_max = 10
        self.PID_integral_min = -10
        self.ctrl_dt = 0.1

        try:
            rospy.loginfo("Experiment: {}".format(time.time()))
            
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



    def main(self):
        rospy.init_node('deploy_node', anonymous=True)

        self.configure_logging()

        sub_startpoint = rospy.Subscriber("/kingfisher/agiros_pilot/state", agiros_msgs.QuadState, self.callback_state)

        # sub_aruco = rospy.Subscriber("/leader_waypoint", geometry_msgs.Point, callback_target)
        sub_aruco = rospy.Subscriber("/fake_waypoint", geometry_msgs.Point, self.callback_target)

        quad_namespace = "kingfisher"
        self.pub = rospy.Publisher(quad_namespace+"/agiros_pilot/velocity_command", 
                        geometry_msgs.TwistStamped, 
                        queue_size=1)

        rate = rospy.Rate(1)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Process callbacks and wait for messages
            rospy.spin()
            # # Sleep to control loop rate
            # rate.sleep()
        
    def configure_logging(self):
        # Create a logger
        logger = logging.getLogger('rosout')
        logger.setLevel(logging.INFO)

        # Create a file handler
        log_file = 'logfile_deploy.log'  # Update this path
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
        self.target_poses.append( np.array([data.x, data.y, data.z]))

        if len(self.target_poses) >= self.target_queue:
            self.target_queue.pop(0)

    def control_scheduler(self):
        while not self.stop_event.is_set():
            # check for new target points
            if len(self.target_poses) > 0:
                active_target_pose = np.copy(self.target_poses[-1])

                integral_error = np.array([0.0, 0.0, 0.0])
                previous_error = np.array([0.0, 0.0, 0.0])

                rospy.loginfo("Starting velPID ctrl, curent pos: {}, target pos: {}".format(self.current_pose, active_target_pose))

                while (not self.stop_event.is_set() 
                    and self.current_pose is not None 
                    and np.array_equal(active_target_pose, self.target_poses[-1] )
                    and not self._reached_target_position(self.current_pose, active_target_pose)):
                    
                    target_pose = self.target_poses[-1]
    
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
                    
                    print("cmd", velocity_command)
                    velocity_command = self._limitVelocity(velocity_command)

                    # Send velocity commands to the quadcopter
                    print("cmd", velocity_command)
                    vel_cmd = geometry_msgs.TwistStamped()
                    vel_cmd.twist.linear = Vector3(0.0, velocity_command[1], 0.0)
                    vel_cmd.twist.angular = Vector3(0.0, 0.0, 0.0)

                    self.pub.publish(vel_cmd)
                    rospy.loginfo("Publishing VelY to Ctrl: {}, Current Pos :{}".format(velocity_command[1], self.current_pose))

                    # Update previous error
                    previous_error = position_error
                    rospy.sleep(self.ctrl_dt)  # Sleep for a while to simulate work

                rospy.loginfo("Exiting velPID loop")

    def _reached_target_position(self, current_position, target_position):
        position_tolerance = 0.1
        return la.norm(current_position - target_position) < position_tolerance

    def _limitVelocity(self, velocity):
        # Only check y now
        max_velocity = 3.0
        velocity[1] = np.clip(velocity[1], -max_velocity, max_velocity)
        
        return velocity

    def stop_threads(self):
        self.stop_event.set()
        self.ctrl_thread.join()

if __name__ == '__main__':
    dep = Deploy()
    dep.main()