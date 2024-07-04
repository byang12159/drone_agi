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

class Deploy:
    def __init__(self):
        self.target_pose = None
        self.current_pose = None
        self.pub = None
        self.position_history = []


        # Debug use:
        self.debug_callback_time = None
        self.debug_vel_publish_time = None
        self.time_record = []
        self.time_start = time.time()


        rospy.init_node('deploy_node', anonymous=True)
        self.pub_start = rospy.Publisher('/start_autonomy', Bool, queue_size=1)
        self.configure_logging()

        self.sub_aruco = rospy.Subscriber("/leader_waypoint", geometry_msgs.Point, self.callback_target)
        self.sub_startpoint = rospy.Subscriber("/kingfisher/agiros_pilot/state", agiros_msgs.QuadState, self.callback_state)

        # Flight with vision
        # self.sub_aruco = rospy.Subscriber("/leader_waypoint", geometry_msgs.Point, self.callback_target)
        # Debugging with fake_cam publisher
        # self.sub_aruco = rospy.Subscriber("/fake_waypoint", geometry_msgs.Point, self.callback_target)



        
     
        self.pub_PF = rospy.Publisher("/leader_global", geometry_msgs.Point, queue_size=1)


        self.rate = rospy.Rate(40)  #Hz


    def main(self):
        rospy.loginfo("Experiment: {}".format(self.time_start))
        print("Ready for Tracking ......")


        for j in range(5):
           self.pub_start.publish(True)
        print("finished publishing start signal")
        # self.pub_start.unregister()


        while not rospy.is_shutdown():
            # Process callbacks and wait for messages
            rospy.spin()
            # # Sleep to control loop rate
            self.rate.sleep()

    def callback_target(self, data):
        self.target_pose = np.array([data.x+self.current_pose[0], data.y+self.current_pose[1], data.z+self.current_pose[2]])
        
        displacement_msg = Point()
        displacement_msg.x = self.target_pose[0]
        displacement_msg.y = self.target_pose[1]
        displacement_msg.z = self.target_pose[2]
        self.pub_PF.publish(displacement_msg)
        rospy.loginfo("Published PF Point message: {}".format(displacement_msg))
 

    def callback_state(self, data):
       self.current_pose = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
       # rospy.loginfo("Recieving State: {},{},{}".format(global_x,global_y,global_z))
      
    def configure_logging(self):
       log_file = 'logfile_deploy2.log' 

       # Create a logger
       logger = logging.getLogger('rosout')
       logger.setLevel(logging.INFO)
       # Create a file handler
       fh = logging.FileHandler(log_file)
       fh.setLevel(logging.INFO)
       # Create a formatter and set it for the handler
       formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
       fh.setFormatter(formatter)
       # Add the file handler to the logger
       logger.addHandler(fh)

if __name__ == '__main__':
    dep = Deploy()

    try:
        dep.main()
    except rospy.ROSInterruptException:
            rospy.loginfo("ROSInterruptException caught")
    finally:
        rospy.loginfo("EXITING Main")
        with open("Fruits.pkl", "wb") as filehandler:
            pickle.dump(dep.position_history, filehandler)
        rospy.loginfo("Finished Script")


        runtimes = np.array(dep.time_record)
        print("datapoints num: ", runtimes.shape)
        print("max: ",np.max(runtimes))
        print("min: ",np.min(runtimes))
        print("std: ",np.std(runtimes))
        print("mean: ",np.mean(runtimes))