#!/usr/bin/env python3
import rospy
from particle_main import RunParticle
import time
import numpy as np
import torch

start_PF = False
particle_state_est=[[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0]]
variance_history = []
PF_history = []
prediction_history = []

def system_callback(data):
    global start_PF 
    start_PF = True

def main():
    global start_PF, particle_state_est, variance_history, PF_history, prediction_history 
    dt = 0.03

    rospy.init_node('particlefilter', anonymous=True)
    rospy.Subscriber('/start_tracking',system_callback,queue_size=1)
    rospy.Subscriber('/state_upate',system_callbac!!!!k,queue_size=1)
    pub = rospy.Publisher("/PF_output", Point, queue_size=1)

    rate = rospy.Rate(1./dt)  #Hz

    while not rospy.is_shutdown() and start_PF == False:
        rate.sleep()
    
    rospy.loginfo('Initialize Particle Filter')

    mcl = RunParticle(starting_state=simple_traj[0])  

    PF_history.append(np.array(mcl.filter.particles['position'].cpu()))

    while not rospy.is_shutdown():

        start_time = time.time()
            
        state_est, variance = mcl.rgb_run(current_pose= simple_traj[iter], past_states1=particle_state_est[-1], time_step=dt )   
        
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