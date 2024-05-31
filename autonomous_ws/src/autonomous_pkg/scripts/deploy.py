#!/usr/bin/env python3
import rospy
import sys
import logging
import time
from std_msgs.msg import Float32, String
import geometry_msgs.msg as geometry_msgs
import agiros_msgs.msg as agiros_msgs

global_x = None
global_y = None
global_z = None

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
    # This function is called every time a new Point message is received
    # Republish the received point message to a new topic

    global global_x, global_y, global_z

    # if data.x != 0.0 and data.y != 0.0 and data.z != 0.0:
    # Aruco Detection        
    # global_x = (data.x - 2.0)
    # global_y += data.y
    # # global_z = data.z + 1.0

    # global_x=0.0
    # global_z=1.0

    # Safety height filter
    # if global_z >= 2.5:
    #     global_z = 2.5
    # if abs(global_y) >= 10.0:
    #     global_y = 10.0
    # if global_x >= 5.0:
    #     global_x = 5.0

    # if abs(data.y) <= 0.05:
    #     pass
    # elif data.y > 0: #move left
    #     global_y += 0.15
    # else: #move right
    #     global_y -= 0.15

    # global_x = 0.0
    # global_y = -1.0
    # global_z = 1.0

    # global_y += data.y
    # # global_z = data.z + 1.0

    # global_x=0.0
    # global_z=1.0
    rospy.loginfo("Recieving State: {},{},{}".format(global_x,global_y,global_z))

    target = geometry_msgs.PoseStamped()
    target.pose.position.x = float(0.0)
    target.pose.position.y = float( global_y + data.y )
    target.pose.position.z = float(1.0) 
    target.pose.orientation.w = float(1.0)
    target.pose.orientation.z = float(0.0)

    pub.publish(target)
    
    # print(("Aruco Pose : {}".format([data.x,data.y,data.z])))
    rospy.loginfo("Aruco Pose : {}".format([data.x,data.y,data.z]))

    rospy.loginfo("Publishing Pose to Ctrl: {}".format([target.pose.position.x,target.pose.position.y, target.pose.position.z]))

    # sub_aruco.unregister()


if __name__ == '__main__':
    try:
        
        
        rospy.init_node('deploy_node', anonymous=True)

        configure_logging()
        rospy.loginfo("Experiment: {}".format(time.time()))

        sub_startpoint = rospy.Subscriber("/kingfisher/agiros_pilot/state", agiros_msgs.QuadState, callback_startpoint)

        sub_aruco = rospy.Subscriber("/leader_waypoint", geometry_msgs.Point, point_callback)

        quad_namespace = "kingfisher"
        pub = rospy.Publisher(quad_namespace+"/agiros_pilot/go_to_pose", 
                            geometry_msgs.PoseStamped, 
                            queue_size=1)

        # Spin to keep the node alive
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
