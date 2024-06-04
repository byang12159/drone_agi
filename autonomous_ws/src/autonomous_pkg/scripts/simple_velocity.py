import rospy
import geometry_msgs.msg as geometry_msgs
from geometry_msgs.msg import TwistStamped, Twist, Vector3
import numpy as np
import numpy.linalg as la
from geometry_msgs.msg import Point
import time


global_x = None
global_y = None
global_z = None
pub = None

def PID_vel(data):
    global pub
    global global_x, global_y, global_z
    # Clear the stop event before starting the loop

    print("INITIALIZE PID VARIABLES")
    k_p = 1.3
    k_i = 0.6
    k_d = 0.0
    integral_max = 10
    integral_min = -10
    

    integral_error = np.array([0.0, 0.0, 0.0])
    previous_error = np.array([0.0, 0.0, 0.0])
    
    dt = 0.1

    target_pos =np.array([data.x, data.y, data.z]) 
    current_pos = np.array([np.copy(global_x),np.copy(global_y),np.copy(global_z)])

    rospy.loginfo("Starting PID_vel function, curent pos: {}, target pos: {},{},{}".format(current_pos, data.x,data.y,data.z))

    while not reached_target_position(current_pos, target_pos):
        
        current_pos = np.array([np.copy(global_x),np.copy(global_y),np.copy(global_z)])

        # Position Error
        position_error = target_pos - current_pos
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

def publish_velocity_command():

    rospy.init_node('velocity_command_publisher', anonymous=True)

    quad_namespace = "kingfisher"
    pub = rospy.Publisher(quad_namespace+"/agiros_pilot/velocity_command", 
                        geometry_msgs.TwistStamped, 
                        queue_size=1)
    
    rate = rospy.Rate(10) # 10hz
    count = 0

    while not rospy.is_shutdown() and count <=5:   

        if count%2 == 0:
            displacement_msg = Point()
            displacement_msg.x =  0.0
            displacement_msg.y = 0.5
            displacement_msg.z = 1.0

            PID_vel(displacement_msg)

        else:
            displacement_msg = Point()
            displacement_msg.x =  0.0
            displacement_msg.y = -0.5
            displacement_msg.z = 1.0

            PID_vel(displacement_msg)

        count +=1
        time.sleep(5)

if __name__ == '__main__':
    try:
        publish_velocity_command()
    except rospy.ROSInterruptException:
        pass

    print("DONE")
