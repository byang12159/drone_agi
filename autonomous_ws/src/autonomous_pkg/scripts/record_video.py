import cv2
import time
import os

# Set up Arducam video capture (modify this based on your Arducam interface)
# Example: For a USB camera, use cv2.VideoCapture(0) or adjust the index as needed
# For other interfaces (e.g., SPI), you may need different initialization code
cap = cv2.VideoCapture(0)  # Modify the index based on your Arducam interface

if not cap.isOpened():
    print("Error: Could not open Arducam.")
    exit()

img_width = 640
img_height = 480
frame_rate = 60
cap.set(2, img_width)
cap.set(4, img_height)
# cap.set(5, frame_rate)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
time.sleep(3)

# Output directory where frames will be saved
output_dir = 'captured_frames2'
os.makedirs(output_dir, exist_ok=True)  # Create the directory if it doesn't exist

# Capture images for 3 seconds
start_time = time.time()
frame_count = 0
while (time.time() - start_time) < 3:
    print(time.time())
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image.")
        break

    # Save the frame as an image
    filename = f"{output_dir}/frame_{frame_count}.jpg"
    cv2.imwrite(filename, frame)
    # print(f"Saved {filename}")

    frame_count += 1

    # Display the frame if needed
    # cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release everything when done
cap.release()
cv2.destroyAllWindows()
