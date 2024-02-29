import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import numpy as np
import sys
import time

try:
    from simple_pid import PID
except Exception as e:
    PID = None

def distance(x, y):
    e = (np.array(x) - np.array(y))
    return np.sqrt((e ** 2).sum())




class Drone:
    def __init__(self):
        # subscribers
        self.state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.evasion_break = False
        self.latest_vicon = 0.
        rospy.Subscriber('/vicon_estimate', Float64MultiArray, self.current_state_callback, queue_size=1)
        rospy.Subscriber('/gate_evasion_check', Bool, self.evasion_callback, queue_size=1)

        self.pub_setpoint_raw = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

        self.Acc_PositionTarget_msg = PositionTarget()
        self.Acc_PositionTarget_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.Acc_PositionTarget_msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_YAW
        self.Acc_PositionTarget_msg.position.x = 0.
        self.Acc_PositionTarget_msg.position.y = 0.
        self.Acc_PositionTarget_msg.position.z = 0.
        self.Acc_PositionTarget_msg.velocity.x = 0.
        self.Acc_PositionTarget_msg.velocity.y = 0.
        self.Acc_PositionTarget_msg.velocity.z = 0.
        self.Acc_PositionTarget_msg.yaw = 0.

        self.Vel_PositionTarget_msg = PositionTarget()
        self.Vel_PositionTarget_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.Vel_PositionTarget_msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW
        self.Vel_PositionTarget_msg.position.x = 0.
        self.Vel_PositionTarget_msg.position.y = 0.
        self.Vel_PositionTarget_msg.position.z = 0.
        self.Vel_PositionTarget_msg.acceleration_or_force.x = 0.
        self.Vel_PositionTarget_msg.acceleration_or_force.y = 0.
        self.Vel_PositionTarget_msg.acceleration_or_force.z = 0.
        self.Acc_PositionTarget_msg.yaw = 0.

        # rospy.wait_for_service("/mavros/cmd/arming")
        # self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

        # rospy.wait_for_service("/mavros/set_mode")
        # self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    def current_state_callback(self, data):
        for i in range(len(self.state)):
            self.state[i] = data.data[i]
        self.latest_vicon = data.data[-1]

    def evasion_callback(self,data):
        # rospy.loginfo("Recieved boolean message %s", msg.data)
        if data.data == True:
            # rospy.loginfo("gate evasion message recieved")
            self.evasion_break = True
            print("changed variable evasion to ",self.evasion_break)
            # rospy.signal_shutdown("Recieved evasion")

    def set_vel(self, vx, vy, vz):
        self.Vel_PositionTarget_msg.velocity.x = vx
        self.Vel_PositionTarget_msg.velocity.y = vy
        self.Vel_PositionTarget_msg.velocity.z = vz
        self.Acc_PositionTarget_msg.yaw_rate = 0.
        self.pub_setpoint_raw.publish(self.Vel_PositionTarget_msg)

    def set_acc(self, ax, ay, az, yaw_rate):
        self.Acc_PositionTarget_msg.acceleration_or_force.x = ax
        self.Acc_PositionTarget_msg.acceleration_or_force.y = ay
        self.Acc_PositionTarget_msg.acceleration_or_force.z = az
        self.Acc_PositionTarget_msg.yaw_rate = yaw_rate
        self.pub_setpoint_raw.publish(self.Acc_PositionTarget_msg)

    def brake(self, velocity_tolerance=0.1, dt=0.02):
        rate = rospy.Rate(1. / dt)
        while (not rospy.is_shutdown()) and (distance(self.state[3:6], [0, 0, 0]) > velocity_tolerance):
            self.set_vel(0, 0, 0)
            rate.sleep()

    def goto(self, position, dt=0.02, tolerance=0.3, velocity_stop=0.1, velocity_tolerance=0.1, verbose=False, brake=False):
        pidx = PID(0.5, 0.1, 0.05, setpoint=position[0])
        pidy = PID(0.5, 0.1, 0.05, setpoint=position[1])
        pidz = PID(0.5, 0.1, 0.05, setpoint=position[2])

        rate = rospy.Rate(1. / dt)
        print("running goto")
        while (not self.evasion_break) and (not rospy.is_shutdown()) and (distance(self.state[:3], position) > tolerance or distance(self.state[3:6], [0, 0, 0]) > velocity_stop):
            # print(self.state[:3])
            # print("1:",self.evasion_break)
            # if evasion_break:
            #     print("detected true")
            #     return
            # print("running goto",evasion_break)
            vx = pidx(self.state[0])
            vy = pidy(self.state[1])
            vz = pidz(self.state[2])
            if verbose:
                print('current_position: (%.3f, %.3f, %.3f); target_position: (%.3f, %.3f, %.3f); v: (%.3f, %.3f, %.3f)'%(self.state[0], self.state[1], self.state[2], position[0], position[1], position[2], vx, vy, vz))
            self.set_vel(vx, vy, vz)
            rate.sleep()

        if brake:
            self.brake(velocity_tolerance, dt)

    def arm(self):
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        if(self.arming_client.call(arm_cmd).success == True):
            rospy.loginfo("Armed!!!")

    def disarm(self):
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = False
        if(self.arming_client.call(arm_cmd).success == True):
            rospy.loginfo("Disarmed!")

    def guided_mode(self):
        # switch to guided mode
        mode_cmd = SetModeRequest()
        mode_cmd.custom_mode = 'GUIDED'
        if(self.set_mode_client.call(mode_cmd).mode_sent == True):
            rospy.loginfo("GUIDED mode enabled")

    def loiter_mode(self):
        # switch to loiter mode
        mode_cmd = SetModeRequest()
        mode_cmd.custom_mode = 'LOITER'
        if(self.set_mode_client.call(mode_cmd).mode_sent == True):
            rospy.loginfo("LOITER mode enabled")