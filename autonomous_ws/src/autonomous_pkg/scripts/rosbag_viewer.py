import rosbag

with rosbag.Bag('my_data.bag', 'r') as bag:
    for topic, msg, t in bag.read_messages():
        print(f"Topic: {topic}, Message: {msg}")