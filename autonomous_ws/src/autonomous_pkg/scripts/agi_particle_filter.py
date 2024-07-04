#!/usr/bin/env python3
import rospy
from particle_main import RunParticle
import time
import numpy as np
import torch
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Point
from std_msgs.msg import Float32, String, Bool

start_PF = False
particle_state_est=[[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0]]
variance_history = []
PF_history = []
prediction_history = []
state_current = None
pub_startsignal = None

# def system_callback(data):
#     global start_PF, pub_startsignal
#     start_PF = True
#     # pub_startsignal.unregister()

def callback_global_position(data):
    global state_current
    print("CALLBACK dat ",data)
    state_current = np.array([data.x, data.y, data.z])

def main():
    global start_PF, particle_state_est, variance_history, PF_history, prediction_history, state_current, pub_startsignal
    dt = 0.03

    rospy.init_node('particlefilter', anonymous=True)
    print("Waiting")
    rospy.wait_for_message('/leader_global', Point)
    print("Recieved")

    print("state current",state_current)
    # rospy.wait_for_message('/start_autonomy', Bool)
    # sub_startsignal = rospy.Subscriber('/start_autonomy',Bool, system_callback,queue_size=3)

    rospy.Subscriber('/leader_global',Point, callback_global_position,queue_size=1)
    pub = rospy.Publisher("/PF_output", Point, queue_size=1)

    rate = rospy.Rate(1./dt)  #Hz

    # state_current = np.array([0,1,2])

    while state_current is None:
        # rospy.spin()
        rate.sleep()
    
    rospy.loginfo('Initialize Particle Filter')
    print("starting PF")

    mcl = RunParticle(starting_state=state_current)  

    PF_history.append(np.array(mcl.filter.particles['position'].cpu()))

    print("starting PF2")
    while not rospy.is_shutdown():

        start_time = time.time()
            
        state_est, variance = mcl.rgb_run(current_pose= state_current, past_states1=particle_state_est[-1], time_step=dt )   
        
        displacement_msg = Point()
        displacement_msg.x = state_est[0]
        displacement_msg.y = state_est[1]
        displacement_msg.z = state_est[2]
        pub.publish(displacement_msg)
        rospy.loginfo("Published PF Point message: {}".format(displacement_msg))

        particle_state_est.append(state_est.cpu().numpy())
        variance_history.append(variance)
        PF_history.append(np.array(mcl.filter.particles['position'].cpu()))
        
        rate.sleep()
# def main():
#     global start_PF, particle_state_est, variance_history, PF_history, prediction_history, state_current, pub_startsignal
#     dt = 0.03

#     rospy.init_node('particlefilter', anonymous=True)
#     print("Waiting for leader global position")
#     rospy.wait_for_message('/leader_global', Point)
#     print("Received leader global position")

#     rospy.Subscriber('/leader_global', Point, callback_global_position, queue_size=1)
#     pub = rospy.Publisher("/PF_output", Point, queue_size=1)

#     rospy.loginfo('Initialize Particle Filter')
#     print("Initializing Particle Filter")

#     # Initialize your particle filter instance
#     while state_current is None:
#         rospy.sleep(0.1)  # Let the state_current be updated
#     mcl = RunParticle(starting_state=state_current)

#     def publish_loop(event):
#         global state_current, mcl
#         state_est, variance = mcl.rgb_run(current_pose=state_current, past_states1=particle_state_est[-1], time_step=dt)

#         displacement_msg = Point()
#         displacement_msg.x = state_est[0]
#         displacement_msg.y = state_est[1]
#         displacement_msg.z = state_est[2]
#         pub.publish(displacement_msg)
#         rospy.loginfo("Published PF Point message: {}".format(displacement_msg))

#         particle_state_est.append(state_est.cpu().numpy())
#         variance_history.append(variance)
#         PF_history.append(np.array(mcl.filter.particles['position'].cpu()))

#     rospy.Timer(rospy.Duration(dt), publish_loop)

#     rospy.spin()



if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass