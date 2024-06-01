import numpy.linalg as la
import rospy
import geometry_msgs.msg as geometry_msgs
from geometry_msgs.msg import TwistStamped, Twist, Vector3

def PID_vel(pub, thread_event, current_position, target_position):

    k_p = 1.0
    k_i = 0.0
    k_d = 0.0

    integral_error = (0, 0, 0)
    previous_error = (0, 0, 0)
    
    dt = 0.1

    while not reached_target_position(current_position, target_position):
        
        # Position Error
        position_error = target_position - current_position
        # Integral Error
        integral_error += position_error * dt
        # Derivative Error
        derivative_error = (position_error - previous_error) / dt

        # Compute PID velocity command
        velocity_command = k_p * position_error + k_i * integral_error + k_d * derivative_error

        limitVelocity(velocity_command)

        # Send velocity commands to the quadcopter
        vel_cmd = geometry_msgs.TwistStamped()
        vel_cmd.twist.linear = Vector3(0.0, velocity_command[1], 0.0)
        vel_cmd.twist.angular = Vector3(0.0, 0.0, 0.0)

        pub.publish(vel_cmd)
        rospy.loginfo("Publishing VelY to Ctrl: {}".format(target_vel_y))

        // Update previous error
        previous_error = position_error;

        
    

def reached_target_position(current_position, target_position):
    position_tolerance = 0.1
    return la.norm(current_position - target_position) < position_tolerance


def limitVelocity(velocity):
    max_velocity = 2.0

    if (velocity > max_velocity):
        velocity = max_velocity
        return
    
    if (velocity < -max_velocity):
        velocity = -max_velocity
        return

