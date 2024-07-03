#!/usr/bin/env python3
import rospy
from particle_main import RunParticle
import time
import numpy as np
import torch
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Point

start_PF = False
particle_state_est=[[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0]]
variance_history = []
PF_history = []
prediction_history = []
state_current = None
pub_startsignal = None

def system_callback(data):
    global start_PF, pub_startsignal
    start_PF = True
    pub_startsignal.unregister()

def callback_global_position(data):
    global state_current
    state_current = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

def main():
    global start_PF, particle_state_est, variance_history, PF_history, prediction_history, state_current, pub_startsignal
    dt = 0.03

    rospy.init_node('particlefilter', anonymous=True)
    sub_startsignal = rospy.Subscriber('/start_tracking',system_callback,queue_size=1)
    rospy.Subscriber('/leader_global',callback_global_position,queue_size=1)
    pub = rospy.Publisher("/PF_output", Point, queue_size=1)

    rate = rospy.Rate(1./dt)  #Hz

    while not rospy.is_shutdown() and start_PF == False and state_current is not None:
        rate.sleep()
    
    rospy.loginfo('Initialize Particle Filter')

    mcl = RunParticle(starting_state=state_current)  

    PF_history.append(np.array(mcl.filter.particles['position'].cpu()))

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

if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass