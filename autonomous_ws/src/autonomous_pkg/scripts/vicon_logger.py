import rospy
import pickle
from std_msgs.msg import Float64MultiArray

class ViconSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('vicon_subscriber', anonymous=True)

        # Subscribe to the /vicon_estimate topic
        self.subscriber = rospy.Subscriber('/vicon_estimate', Float64MultiArray, self.callback)
        self.subscriber_l = rospy.Subscriber('/vicon_estimate_leader', Float64MultiArray, self.callback_leader)

        # Initialize an empty list to store data
        self.data = []
        self.data_l =[]

        # Specify the filename for the pickle file
        self.pickle_file = 'vicon_logger_P0_agi_2.pkl'
        self.pickle_file_l = 'vicon_logger_P0_lead_2.pkl'

    def callback(self, msg):
        # Append the received data to the list
        self.data.append(msg.data)

        # Save the data to a pickle file
        with open(self.pickle_file, 'wb') as f:
            pickle.dump(self.data, f)

        rospy.loginfo(f"Data saved: {msg.data}")
    
    def callback_leader(self, msg):
        self.data_l.append(msg.data)

        # Save the data to a pickle file
        with open(self.pickle_file_l, 'wb') as f:
            pickle.dump(self.data_l, f)

        rospy.loginfo(f"Data saved2: {msg.data}")

    def spin(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        vicon_subscriber = ViconSubscriber()
        vicon_subscriber.spin()
    except rospy.ROSInterruptException:
        pass
