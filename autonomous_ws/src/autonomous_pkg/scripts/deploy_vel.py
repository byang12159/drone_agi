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

global_x = None
global_y = None
global_z = None
pub = None

def configure_logging():
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

def callback_startpoint(data):
    # function called once at initialization
    global global_x, global_y, global_z

    global_x = data.pose.position.x
    global_y = data.pose.position.y
    global_z = data.pose.position.z
    
    # rospy.loginfo("Recieving State: {},{},{}".format(global_x,global_y,global_z))
    
 
def point_callback(data):
    global pub
    # This function is called every time a new Point message is received
    # Republish the received point message to a new topic

    global global_x, global_y, global_z

    temp_state_x = np.copy(global_x)
    temp_state_y = np.copy(global_y)
    temp_state_z = np.copy(global_z)
    
    rospy.loginfo("Recieving State: {},{},{}".format(temp_state_x, temp_state_y, temp_state_z))

    if data.y >=2.0:
        target_vel_y = 2.0
    elif data.y <= -2.0:
        target_vel_y = -2.0
    else:
        target_vel_y = data.y

    vel_cmd = geometry_msgs.TwistStamped()
    vel_cmd.twist.linear = Vector3(0.0, target_vel_y, 0.0)
    vel_cmd.twist.angular = Vector3(0.0, 0.0, 0.0)

    pub.publish(vel_cmd)
    
    # print(("Aruco Pose : {}".format([data.x,data.y,data.z])))
    rospy.loginfo("Aruco Pose : {}".format([data.x,data.y,data.z]))

    rospy.loginfo("Publishing VelY to Ctrl: {}".format(target_vel_y))

    # sub_aruco.unregister()

def main():
    global pub

    rospy.init_node('deploy_node', anonymous=True)

    configure_logging()

    rospy.loginfo("Experiment: {}".format(time.time()))

    sub_startpoint = rospy.Subscriber("/kingfisher/agiros_pilot/state", agiros_msgs.QuadState, callback_startpoint)

    sub_aruco = rospy.Subscriber("/leader_waypoint", geometry_msgs.Point, point_callback)

    quad_namespace = "kingfisher"
    pub = rospy.Publisher(quad_namespace+"/agiros_pilot/velocity_command", 
                    geometry_msgs.TwistStamped, 
                    queue_size=1)
    

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Process callbacks and wait for messages
        rospy.spin()

        # Sleep to control loop rate
        rate.sleep()



if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass
