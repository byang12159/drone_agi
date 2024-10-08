import cv2
import os

script_dir = os.path.dirname(os.path.realpath(__file__))

# Define the directory containing the images
image_folder = os.path.join(script_dir, './frames')  # Replace with the path to your image folder

# Define the output video file name
video_name = 'output_video.mp4'

# Get the list of all image files in the folder
images = [img for img in os.listdir(image_folder) if img.endswith((".png", ".jpg", ".jpeg"))]
images.sort()  # Sort the images to ensure correct order

# Read the first image to get the video frame size
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, _ = frame.shape

# Define the codec and create a VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for MP4 files
video = cv2.VideoWriter(video_name, fourcc, 10.0, (width, height))  # 30.0 is the frames per second (fps)

# Loop through each image and write it to the video
for image in images:
    frame = cv2.imread(os.path.join(image_folder, image))
    video.write(frame)

# Release the video writer
video.release()

print(f"Video saved as {video_name}")