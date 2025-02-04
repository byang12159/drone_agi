import rospy 
import agiros_msgs.msg as agiros_msgs 
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Point, Pose, PointStamped, PoseArray
import numpy as np 
from std_msgs.msg import Float32, String, Bool,Float64MultiArray
from pyquaternion import Quaternion
import time 

waypoints_list = np.array([])
waypoints_pose_list = []
current_waypoint_id = 0
accumulated_dist_list = np.array([])
active_target_pose = np.array([])
vicon_pose = None
current_pose = None 

def callback_fake(data:PoseArray):
    global waypoints_list, waypoints_pose_list, current_waypoint_id, accumulated_dist_list, active_target_pose 
    # global new_message_received, target_pose, current_pose,accumulated_backoff
    # new_message_received = True

    # target_pose = np.array([data.x, data.y, data.z])
    
    # print("Spotted Marker",target_pose)
    if waypoints_list.size>0:
        # print("rejecting repeated waypoints list")
        return
    # print("message received")
    posearray = data.poses
    # print("Length: {}".format(len(posearray))) 
    waypoints_list_tmp = []
    accumulated_dist_list_tmp = []
    for i in range(len(posearray)):
        pose = posearray[i]
        waypoints_pose_list.append(pose)
        pos = pose.position 
        ori = pose.orientation

        x,y,z = pos.x, pos.y, pos.z 
        qx, qy, qz, qw = ori.x, ori.y, ori.z, ori.w 
        # print("qx, qy, qz, qw",qx, qy, qz, qw)

        # yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        q = Quaternion(qw, qx, qy, qz)
        yaw = q.yaw_pitch_roll[0]

        waypoints_list_tmp.append([x,y,z,yaw])

        if i==0:
            accumulated_dist_list_tmp.append(0)
        else:
            prev_x, prev_y, prev_z = posearray[i-1].position.x,posearray[i-1].position.y,posearray[i-1].position.z
            accumulated_dist_list_tmp.append(
                accumulated_dist_list_tmp[-1]
                + np.linalg.norm([x-prev_x, y-prev_y, z-prev_z]))

    waypoints_list = np.array(waypoints_list_tmp)
    waypoints_list = np.array(waypoints_list)
    current_waypoint_id = 0
    accumulated_dist_list = np.array(accumulated_dist_list_tmp)
    active_target_pose = waypoints_list[0,:]

def callback_state(data):
    global current_pose
    qx, qy, qz, qw = data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w
    # yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    q = Quaternion(qw, qx, qy, qz)
    yaw = q.yaw_pitch_roll[0]

    # print("Current pose: {}, {}, {}, {}".format(data.pose.position.x, data.pose.position.y, data.pose.position.z, yaw))
    current_pose = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z, yaw])

def callback_vicon(data):
    global vicon_pose
    vicon_pose = np.array([data.data[0], data.data[1], data.data[2]])

if __name__ == "__main__":
    rospy.init_node('trajectory_node')

    pub = rospy.Publisher("/kingfisher/agiros_pilot/trajectory", agiros_msgs.Reference, queue_size=1)
    sub_waypoints = rospy.Subscriber('/fake_waypoint_list', PoseArray, callback_fake, queue_size=1)
    sub_vicon = rospy.Subscriber("/vicon_estimate", Float64MultiArray, callback_vicon, queue_size=1)
    sub_startpoint = rospy.Subscriber("/kingfisher/agiros_pilot/state", agiros_msgs.QuadState, callback_state, queue_size=1)

    while waypoints_list.size<=0 or accumulated_dist_list.size<=0 or vicon_pose is None or current_pose is None:
        # print(waypoints_list.size, accumulated_dist_list.size, vicon_pose, current_pose)
        rospy.sleep(0.1) 

    points_list = []
    for i in range(len(waypoints_pose_list)):
        pose = waypoints_pose_list[i]
        quadstate = agiros_msgs.QuadState(pose=pose)
        setpoint = agiros_msgs.Setpoint(state = quadstate)
        points_list.append(setpoint)

    reference = agiros_msgs.Reference(points_list)
    for i in range(10):
        print("sending trajectory")
        pub.publish(reference)
        time.sleep(1)