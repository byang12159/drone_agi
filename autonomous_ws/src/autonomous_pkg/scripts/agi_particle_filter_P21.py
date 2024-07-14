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
MC_prediction_on = None
MC_prediction_last = False #assume detection first
mode_switch = 0
last_prediction_signal = False
mode_predict =  True
mode_backoff = False
prediction_count = 0

# def system_callback(data):
#     global start_PF, pub_startsignal
#     start_PF = True
#     # pub_startsignal.unregister()

def callback_state(data):
    global current_pose
    current_pose = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

def callback_leader(data):
    global target_pose, current_pose
    if current_pose is not None:
        target_pose = np.array([data.x+current_pose[0], data.y+current_pose[1], data.z+current_pose[2]])

def callback_detection_bool(data):
    global MC_prediction_on, MC_prediction_last, mode_switch, prediction_count
    if data.data:
        MC_prediction_on = False
        prediction_count = 0 
    else:
        MC_prediction_on = True
        prediction_count +=1

    # if MC_prediction_last!=MC_prediction_on:
    #     mode_switch +=1
    # MC_prediction_last=MC_prediction_on

def main():
    global start_PF, mode_predict,prediction_count,mode_backoff,mode_switch,particle_state_est, variance_history, PF_history, prediction_history, current_pose, pub_startsignal, target_pose, MC_prediction_on, last_prediction_signal

    rospy.init_node('particlefilter', anonymous=True)

    print("Waiting")
    rospy.wait_for_message('/kingfisher/agiros_pilot/state', agiros_msgs.QuadState)
    rospy.wait_for_message('/leader_waypoint', Point)
    print("Recieved")

    sub_detection_bool = rospy.Subscriber('/aruco_detection',Bool, callback_detection_bool,queue_size=3)
    sub_state = rospy.Subscriber("/kingfisher/agiros_pilot/state", agiros_msgs.QuadState, callback_state, queue_size=3)
    sub_aruco_waypoint = rospy.Subscriber('/leader_waypoint',Point, callback_leader,queue_size=3)
    pub = rospy.Publisher("/prediction_output", Point, queue_size=2)

    rate = rospy.Rate(60)  #Hz
    dt = 1.0/60.0
    # rate = rospy.Rate(30)  #Hz
    # dt = 1.0/30.0

    prediction = Prediction()

    while current_pose is None or target_pose is None:
        rate.sleep()
    
    particle_state_est.append(np.array([target_pose[0], target_pose[1], target_pose[2],0.0,0.0,0.0]))
    
    rospy.loginfo('Initialize Particle Filter')
    mcl = RunParticle(starting_state=target_pose)  
    PF_history.append(mcl.filter.particles['position'])

    while not rospy.is_shutdown():

        start_time = time.time()

        if MC_prediction_on == False:
            state_est, variance = mcl.rgb_run(current_pose= target_pose, past_states1=particle_state_est[-1], time_step=dt )   
            state_est = state_est.to('cpu').numpy()
            particle_state_est.append(state_est)
            PF_history.append(mcl.filter.particles['position'])
            print("stateest", state_est)
        else:
            if prediction_count >= 10:
                last_state = particle_state_est[-1]
                backoff_depth = prediction.find_prediction(last_state,current_pose,timestep = 0.5,accel_range=5,steps=3)
                print("backoff depth",backoff_depth)
                if backoff_depth != 0.0:
                    displacement = backoff_depth-1.5 

                    if displacement >= 1.2:
                        displacement = 1.2

                    displacement_msg = Point()
                    displacement_msg.x = -displacement
                    displacement_msg.y = last_state[1]
                    displacement_msg.z = 0
                    pub.publish(displacement_msg)
                    print(displacement_msg)


        print("runtime: ",time.time()-start_time, "prediction on? ",MC_prediction_on, "mode_predict:",mode_predict)
        
        rate.sleep()

if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass