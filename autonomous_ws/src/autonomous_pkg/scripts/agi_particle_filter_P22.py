#!/usr/bin/env python3
import rospy
from particle_main import RunParticle
from agi_prediction import Prediction
import time
import numpy as np
import torch
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Point, Pose, PointStamped
from std_msgs.msg import Float32, String, Bool, Float64MultiArray
import agiros_msgs.msg as agiros_msgs

# torch.manual_seed(42)

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
mode_recovery =  False
prediction_count = 0
mode2_count = 0

leader_hist = []
mode = 1

# vicon_pose = None
# def callback_vicon(data):
#     global vicon_pose
#     vicon_pose = np.array([data.data[0], data.data[1], data.data[2]])

def callback_state(data):
    global current_pose
    current_pose = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

def callback_leader(data):
    global target_pose, current_pose
    timestamp = data.header.stamp.to_sec()

    if current_pose is not None:
        new_target_pose = [data.point.x+current_pose[0], data.point.y+current_pose[1], data.point.z+current_pose[2], timestamp]

    append_history(np.array(new_target_pose))
    target_pose = np.array(new_target_pose[0:3])

def callback_detection_bool(data):
    global MC_prediction_on, MC_prediction_last, mode_switch, prediction_count, mode_recovery
    if data.data:
        MC_prediction_on = False
        prediction_count = 0 
        
    else:
        MC_prediction_on = True
        if mode != 4:
            prediction_count +=1

    # if MC_prediction_last!=MC_prediction_on:
    #     mode_switch +=1
    # MC_prediction_last=MC_prediction_on

def append_history(new_pos):
    global leader_hist
    if len(leader_hist) >= 10:
            leader_hist.pop(0)  # Remove the oldest entry
    leader_hist.append(new_pos)
    
def main():
    global start_PF, mode_recovery,prediction_count,mode_switch,particle_state_est, variance_history, PF_history, prediction_history, current_pose, pub_startsignal, target_pose, MC_prediction_on, last_prediction_signal
    global vicon_pose, mode, mode2_count

    for i in range(10):
        append_history(np.array([0,0,0]))

    rospy.init_node('particlefilter', anonymous=True)

    print("Waiting for ego and target state...")
    rospy.wait_for_message('/kingfisher/agiros_pilot/state', agiros_msgs.QuadState)
    rospy.wait_for_message('/leader_waypoint', PointStamped)
    print("Recieved Information")

    sub_detection_bool = rospy.Subscriber('/aruco_detection',Bool, callback_detection_bool,queue_size=1)
    sub_state = rospy.Subscriber("/kingfisher/agiros_pilot/state", agiros_msgs.QuadState, callback_state, queue_size=1)
    sub_aruco_waypoint = rospy.Subscriber('/leader_waypoint',PointStamped, callback_leader,queue_size=1)
    pub = rospy.Publisher("/prediction_output", Pose, queue_size=1)
    # sub_vicon = rospy.Subscriber("/vicon_estimate", Float64MultiArray, callback_vicon, queue_size=5)

    pub_log = rospy.Publisher('/log_messages_PF', Float64MultiArray, queue_size=10)

    rate = rospy.Rate(40)  #Hz
    dt = 1.0/40.0

    log_data = Float64MultiArray()
    log_data.data = [0, ] * (6 + 2 + 3 + 2)

    prediction = Prediction(100, 5)

    while current_pose is None or target_pose is None:
        rate.sleep()
    
    particle_state_est.append(np.array([target_pose[0], target_pose[1], target_pose[2],0.0, 0.0, 0.0]))
    
    rospy.loginfo('Initializing Particle Filter')
    mcl = RunParticle(starting_state=target_pose)  
    PF_history.append(mcl.filter.particles['position'])

    i = 0
    sleep_count = 0

    while not rospy.is_shutdown():

        start_time = time.time()
        backoff_state = [0,0,0]

        if mode == 1:
            if MC_prediction_on and prediction_count >=30:
                mode = 3 
        elif mode == 3:
            if not MC_prediction_on:
                mode = 4
        elif mode==4:
            if abs(current_pose[0] - target_pose[0]) <= 1.8:
                mode=1
        else:
            raise ValueError("Mode {}".format(mode))
        

        if mode==1 and not MC_prediction_on:
            displacement = 1.5
            displacement_msg = Pose()
            displacement_msg.position.x = target_pose[0]-current_pose[0]-displacement
            displacement_msg.position.y = target_pose[1]-current_pose[1]
            displacement_msg.position.z = target_pose[2]-current_pose[2]
            displacement_msg.orientation.x = mode
            pub.publish(displacement_msg)
        
            state_est, variance = mcl.rgb_run(current_pose= target_pose, past_states1=particle_state_est[-1], time_step=dt )   
            state_est = state_est.to('cpu').numpy()
            particle_state_est.append(state_est)
            PF_history.append(mcl.filter.particles['position'])
            print("state-est", state_est)
            mode_recovery = False
     

        elif mode==2:
            # break
            last_state = particle_state_est[-1]
            displacement_msg = Pose()
            displacement_msg.position.x = 0
            displacement_msg.position.y = last_state[4]*20*0.01 # Move chaser using vel-est of leader
            displacement_msg.position.z = 0 #last_state[5]*10*0.01
            displacement_msg.orientation.x = mode
            pub.publish(displacement_msg)
            print("============== Mode 2: Prediction # {} \n Displacement: {}".format(prediction_count,displacement_msg ))
        elif mode==3:
            # Only Publish once for mode 3 

            if mode_recovery:
                pass
            else:
                # last_state = particle_state_est[-1]
                # Use finite difference for vel
                print("LEADER",leader_hist)

                last_vel = (leader_hist[-1][0:3]-leader_hist[-2][0:3])/float(leader_hist[-1][-1]-leader_hist[-2][-1])
                last_state = np.array([leader_hist[-1][0], leader_hist[-1][1], leader_hist[-1][2], 
                                       last_vel[0], last_vel[1], last_vel[2] ])
                print("LASTSTATE",last_state)
                total_trajectories, backoff_state, rectangles = prediction.compute_reach(last_state, timestep = 0.3,accel_range=2)
                displacement_msg = Pose()
                displacement_msg.position.x = backoff_state[0]
                displacement_msg.position.y = backoff_state[1]
                displacement_msg.position.z = 0 # backoff_state[2]
                displacement_msg.orientation.x = mode
                pub.publish(displacement_msg)
                print("============== Mode 3:  Prediction # {}".format(prediction_count))
                print(last_state)
                print(backoff_state)
                print(rectangles[-1,:,:])
                print(total_trajectories[0,0,:])

                mode_recovery = True
        elif mode==4:
            if sleep_count <50:

                displacement_msg = Pose()
                displacement_msg.position.x = 0
                displacement_msg.position.y = 0
                displacement_msg.position.z = 0
                displacement_msg.orientation.x = mode
                pub.publish(displacement_msg)
              
            else:
                displacement = 1.5
                displacement_msg = Pose()
                displacement_msg.position.x = target_pose[0]-current_pose[0]-displacement
                displacement_msg.position.y = target_pose[1]-current_pose[1]
                displacement_msg.position.z = target_pose[2]-current_pose[2]
                displacement_msg.orientation.x = mode
                pub.publish(displacement_msg)
            
            sleep_count +=1
                
            state_est, variance = mcl.rgb_run(current_pose= target_pose, past_states1=particle_state_est[-1], time_step=dt )   
            state_est = state_est.to('cpu').numpy()
            particle_state_est.append(state_est)
            PF_history.append(mcl.filter.particles['position'])
            
            mode_recovery = False
            
            print("============== Mode 4")
            print("state-est", target_pose)
        elif mode not in [1,2,3,4]:
            raise ValueError("Mode {}".format(mode))
    
        
        # ROS Logging 
        log_data.data[0] = particle_state_est[-1][0]
        log_data.data[1] = particle_state_est[-1][1]
        log_data.data[2] = particle_state_est[-1][2]
        log_data.data[3] = particle_state_est[-1][3]
        log_data.data[4] = particle_state_est[-1][4]
        log_data.data[5] = particle_state_est[-1][5]
        log_data.data[6] = mode
        log_data.data[7] = prediction_count
        log_data.data[8] = backoff_state[0]
        log_data.data[9] = backoff_state[1]
        log_data.data[10] = backoff_state[2]
        log_data.data[-2] = time.time()-start_time # While loop runtime
        now = rospy.get_rostime()
        now = now.to_sec()
        log_data.data[-1] = now
       
        pub_log.publish(log_data)
        # print("Prediction Count: ",prediction_count)
        # print("runtime: ",time.time()-start_time)
        rate.sleep()
        # print("leader",leader_hist)
if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass