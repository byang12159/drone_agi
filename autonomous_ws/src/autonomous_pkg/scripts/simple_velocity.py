import rospy
import geometry_msgs.msg as geometry_msgs
from geometry_msgs.msg import TwistStamped, Twist, Vector3

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
            vel_cmd = geometry_msgs.TwistStamped()
            vel_cmd.twist.linear = Vector3(0.0, 0.8, 0.0)
            vel_cmd.twist.angular = Vector3(0.0, 0.0, 0.0)

        else:
            vel_cmd = geometry_msgs.TwistStamped()
            vel_cmd.twist.linear = Vector3(0.0, -0.8, 0.0)
            vel_cmd.twist.angular = Vector3(0.0, 0.0, 0.0)


        vel_cmd.header.stamp = rospy.Time.now()
        pub.publish(vel_cmd)

        rate.sleep()
        count +=1

if __name__ == '__main__':
    try:
        publish_velocity_command()
    except rospy.ROSInterruptException:
        pass

    print("DONE")
