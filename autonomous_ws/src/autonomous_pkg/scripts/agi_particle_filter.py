#!/usr/bin/env python3
import rospy
from particle_main import RunParticle
from agi_prediction import Prediction
import time
import numpy as np
import torch
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Point
from std_msgs.msg import Float32, String, Bool
import agiros_msgs.msg as agiros_msgs

torch.manual_seed(42)

start_PF = False
particle_state_est=[]
variance_history = []
PF_history = []
prediction_history = []
current_pose = None
target_pose = None
pub_startsignal = None
prediction_on = None
last_prediction_signal = False

# def system_callback(data):
#     global start_PF, pub_startsignal
#     start_PF = True
#     # pub_startsignal.unregister()

def callback_state(data):
    global current_pose
    current_pose = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

def callback_leader(data):
    global target_pose
    target_pose = np.array([data.x+current_pose[0], data.y+current_pose[1], data.z+current_pose[2]])

def callback_detection_bool(data):
    global prediction_on
    if data.data:
        prediction_on = False
    else:
        prediction_on = True

def main():
    global start_PF, particle_state_est, variance_history, PF_history, prediction_history, current_pose, pub_startsignal, target_pose, prediction_on, last_prediction_signal

    rospy.init_node('particlefilter', anonymous=True)

    print("Waiting")
    rospy.wait_for_message('/kingfisher/agiros_pilot/state', agiros_msgs.QuadState)
    rospy.wait_for_message('/leader_waypoint', Point)
    print("Recieved")

    sub_detection_bool = rospy.Subscriber('/aruco_detection',Bool, callback_detection_bool,queue_size=1)
    sub_state = rospy.Subscriber("/kingfisher/agiros_pilot/state", agiros_msgs.QuadState, callback_state, queue_size=3)
    sub_aruco_waypoint = rospy.Subscriber('/leader_waypoint',Point, callback_leader,queue_size=1)
    pub = rospy.Publisher("/PF_output", Point, queue_size=1)

    rate = rospy.Rate(100)  #Hz
    dt = 1.0/100.0
    # rate = rospy.Rate(30)  #Hz
    # dt = 1.0/30.0

    # current_pose = np.array([0,1,2])

    prediction = Prediction()

    while current_pose is None or target_pose is None:
        rate.sleep()
    
    particle_state_est.append(target_pose)
    
    rospy.loginfo('Initialize Particle Filter')

    mcl = RunParticle(starting_state=target_pose)  
    PF_history.append(mcl.filter.particles['position'])

    while not rospy.is_shutdown():

        start_time = time.time()

        if prediction_on:
            lastpoint = particle_state_est[-1]
            displacement_msg = Point()
            displacement_msg.x = lastpoint[3]
            displacement_msg.y = lastpoint[4]
            displacement_msg.z = lastpoint[5]
            pub.publish(displacement_msg)
        else:
            state_est, variance = mcl.rgb_run(current_pose= target_pose, past_states1=particle_state_est[-1], time_step=dt )   
            state_est = state_est.to('cpu').numpy()
            particle_state_est.append(state_est)
            PF_history.append(mcl.filter.particles['position'])

        # if prediction_on and last_prediction_signal == False:
        #     # Only run once
        #     rectangle, total_trajectories = prediction.find_prediction(state_est, current_pose, dt, accel_range=5, steps = 5, num_trajectory = 100)

        #     displacement_msg = Point()
        #     displacement_msg.x = state_est[0]
        #     displacement_msg.y = state_est[1]
        #     displacement_msg.z = state_est[2]
        #     pub.publish(displacement_msg)

        #     last_prediction_signal = True
        #     rospy.loginfo("Published PF Point message: {}".format(displacement_msg))



        print("runtime: ",time.time()-start_time, "prediction on? ",prediction_on)
        
        rate.sleep()

if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass