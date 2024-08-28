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
fn = os.path.join(script_dir, './exp_07-28-17-26.bag')
img_dir = os.path.join(script_dir, './pi_images/')
rosbag_to_csv(fn, script_dir)
