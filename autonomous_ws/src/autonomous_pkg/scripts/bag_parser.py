import rosbag
import csv

input_bag ='bigmarker.bag' 
output_csv = 'bigmarker_data.csv'
# Open the bag file and CSV file

count_true = 0
count_total = 0

with rosbag.Bag(input_bag, 'r') as bag, open(output_csv, 'wb') as csvfile:
    writer = csv.writer(csvfile)
    # Write header row with topic label
    writer.writerow(['Topic', 'Timestamp', 'Data'])
    # Iterate through messages in the bag file
    for topic, msg, t in bag.read_messages(topics=['/aruco_detection']):
        count_total +=1
        # Convert timestamp to nanoseconds for better precision
        timestamp = t.to_nsec()
        data = msg.data if hasattr(msg, 'data') else 'N/A'  # Handle messages without 'data' attribute
        if data == True:
            count_true+=1
        writer.writerow([topic, timestamp, data])

print("Count True: ",count_true)
print("Count Total: ", count_total)
print("Detection %: ",(count_true/count_total)*100)