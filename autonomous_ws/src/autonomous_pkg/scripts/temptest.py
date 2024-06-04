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

global_x = None
global_y = None
global_z = None
target_pose = None
last_target_pose = None
pub = None
stop_event = threading.Event() # Create a global event that can be used to signal the loop to stop


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
    global global_x, global_y, global_z

    global_x = data.pose.position.x
    global_y = data.pose.position.y
    global_z = data.pose.position.z
    
    # rospy.loginfo("Recieving State: {},{},{}".format(global_x,global_y,global_z))
 
def point_callback(data):
    global target_pose, last_target_pose

    last_target_pose = np.copy(target_pose)
    target_pose = np.array([data.x, data.y, data.z])

    rospy.loginfo("Recieve new callback with target {}, last target pose: {}".format(target_pose, last_target_pose))

def PID_vel():
    global pub
    global global_x, global_y, global_z
    global target_pose, last_target_pose
   
    while True:
        k_p = 1.3
        k_i = 0.6
        k_d = 0.0
        integral_max = 10
        integral_min = -10
        dt = 0.1

        integral_error = np.array([0.0, 0.0, 0.0])
        previous_error = np.array([0.0, 0.0, 0.0])
        
        last_target_pose = np.copy(target_pose)

        while target_pose is not None and last_target_pose==target_pose:
            #Checks if target pose is initialized + no new incoming target poses
        
            current_pos = np.array([np.copy(global_x),np.copy(global_y),np.copy(global_z)])

            rospy.loginfo("Starting PID_vel function, curent pos: {}, target pos: {}".format(current_pos, target_pose))


            current_pos = np.array([np.copy(global_x),np.copy(global_y),np.copy(global_z)])

            # Position Error
            position_error = target_pose - current_pos
            # Integral Error
            integral_error += position_error * dt
            integral_error[0] = max(min(integral_error[0], integral_max), integral_min)  # Clamping
            integral_error[1] = max(min(integral_error[1], integral_max), integral_min)
            integral_error[2] = max(min(integral_error[2], integral_max), integral_min)
        
            # Derivative Error
            derivative_error = (position_error - previous_error) / dt

            # Compute PID velocity command
            velocity_command = k_p * position_error + k_i * integral_error + k_d * derivative_error
            
            print("cmd", velocity_command)
            velocity_command = limitVelocity(velocity_command)

            # Send velocity commands to the quadcopter
            print("cmd", velocity_command)
            vel_cmd = geometry_msgs.TwistStamped()
            vel_cmd.twist.linear = Vector3(0.0, velocity_command[1], 0.0)
            vel_cmd.twist.angular = Vector3(0.0, 0.0, 0.0)

            pub.publish(vel_cmd)
            rospy.loginfo("Publishing VelY to Ctrl: {}, Current Pos :{}".format(velocity_command[1], current_pos))
            rospy.loginfo("current interal error: {}".format(integral_error))

            # Update previous error
            previous_error = position_error
            if reached_target_position(current_pos, target_pose):
                rospy.loginfo("Reached target position")
                break

            rospy.sleep(dt)  # Sleep for a while to simulate work

            rospy.loginfo("Exiting PID_vel function")

def reached_target_position(current_position, target_position):
    position_tolerance = 0.1
    return la.norm(current_position - target_position) < position_tolerance

def limitVelocity(velocity):
    # Only check y now
    max_velocity = 3.0

    if (velocity[1] > max_velocity):
        velocity[1] = max_velocity
        
    elif (velocity[1] < -max_velocity):
        velocity[1] = -max_velocity
    
    return velocity

def main():
    global pub

    rospy.init_node('deploy_node', anonymous=True)

    configure_logging()

    rospy.loginfo("Experiment: {}".format(time.time()))

    sub_startpoint = rospy.Subscriber("/kingfisher/agiros_pilot/state", agiros_msgs.QuadState, callback_startpoint)

    # sub_aruco = rospy.Subscriber("/leader_waypoint", geometry_msgs.Point, point_callback)
    sub_aruco = rospy.Subscriber("/fake_waypoint", geometry_msgs.Point, point_callback)

    quad_namespace = "kingfisher"
    pub = rospy.Publisher(quad_namespace+"/agiros_pilot/velocity_command", 
                    geometry_msgs.TwistStamped, 
                    queue_size=1)
    
    

    rate = rospy.Rate(1)  # 10 Hz

    while not rospy.is_shutdown():
        # Process callbacks and wait for messages
        rospy.spin()

        # Sleep to control loop rate
        rate.sleep()



if __name__ == '__main__':
    try:
        t1 = threading.Thread(target=main, name='main thread')
        t2 = threading.Thread(target=PID_vel, name='ctrl thread')
        
        t1.start()
        t2.start()
    
        t1.join()
        t2.join()

    except rospy.ROSInterruptException:
        pass

