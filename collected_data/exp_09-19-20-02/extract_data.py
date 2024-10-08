import rosbag
import pandas as pd
from datetime import datetime
import os
import numpy as np 
import cv2


def rosbag_to_csv(bag_file, output_dir):
    # Open the bag file
    bag = rosbag.Bag(bag_file)
    
    # Create the output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Get the list of topics
    topics = bag.get_type_and_topic_info()[1].keys()
    i = 0
    for topic in topics:
        messages = []
        for topic, msg, t in bag.read_messages(topics=[topic]):
            if topic == '/picam':
                msg_dict = {}
                img_fn = os.path.join(img_dir, f"frame_{i:04d}.png")       
                np_arr = np.frombuffer(msg.data, np.uint8)
                if msg.format == "jpeg":
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
                else:
                    print('Unsupported image format: {}'.format(msg.format))
                    continue
                
                if cv_image is None:
                    print('Failed to decode image at timestamp: {}'.format(t))
                    continue
                success = cv2.imwrite(img_fn, cv_image)
                
                msg_dict['fn'] = img_fn
                msg_dict['timestamp'] = t.to_sec()
                messages.append(msg_dict)
                i+=1
            if topic == '/picam_raw':
                msg_dict = {}
                img_fn = os.path.join(img_raw_dir, f"frame_{i:04d}.png")       
                np_arr = np.frombuffer(msg.data, np.uint8)
                if msg.format == "jpeg":
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                else:
                    print('Unsupported image format: {}'.format(msg.format))
                    continue
                
                if cv_image is None:
                    print('Failed to decode image at timestamp: {}'.format(t))
                    continue
                success = cv2.imwrite(img_fn, cv_image)
                
                msg_dict['fn'] = img_fn
                msg_dict['timestamp'] = t.to_sec()
                messages.append(msg_dict)
                i+=1
            elif topic == '/t265/odom/sample':
                msg_dict = {}
                msg_dict['timestamp'] = t.to_sec()
                data = [
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                    msg.twist.twist.angular.x,
                    msg.twist.twist.angular.y,
                    msg.twist.twist.angular.z,
                ]
                cov = msg.pose.covariance
                msg_dict['data'] = data
                msg_dict['cov'] = cov
                messages.append(msg_dict)
            elif topic == '/kingfisher/agiros_pilot/velocity_command':
                msg_dict = {}
                msg_dict['timestamp'] = t.to_sec()
                data = [
                    msg.twist.linear.x,
                    msg.twist.linear.y,
                    msg.twist.linear.z,
                    msg.twist.angular.x,
                    msg.twist.angular.y,
                    msg.twist.angular.z,
                ]
                msg_dict['data'] = data
                messages.append(msg_dict)
            else:
                # Convert ROS message to dictionary
                msg_dict = {}
                for slot in msg.__slots__:
                    msg_dict[slot] = getattr(msg, slot)
                msg_dict['timestamp'] = t.to_sec()
                messages.append(msg_dict)
        
        # Convert list of dictionaries to pandas DataFrame
        df = pd.DataFrame(messages)
        
        # Write DataFrame to CSV
        csv_file = os.path.join(output_dir, topic.replace('/', '_') + '.csv')
        df.to_csv(csv_file, index=False)
    
    bag.close()

# Usage
script_dir = os.path.dirname(os.path.realpath(__file__))
fn = os.path.join(script_dir, './exp_09-19-20-02.bag')
img_dir = os.path.join(script_dir, './pi_images/')
if not os.path.exists(img_dir):
    os.mkdir(img_dir)
img_raw_dir = os.path.join(script_dir, './pi_images_raw/')
if not os.path.exists(img_raw_dir):
    os.mkdir(img_raw_dir)
rosbag_to_csv(fn, script_dir)
