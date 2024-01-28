import cv2
import os

# Open a video capture object for the USB camera (usually 0 or 1 depending on your setup)
cap = cv2.VideoCapture(0)

# Check if the camera is opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Set the frame width and height (adjust as needed)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Specify the folder to save images
output_folder = "captured_frames_raw"

# Create the output folder if it doesn't exist
if not os.path.exists(output_folder):
    os.makedirs(output_folder)
    print(f"Created folder: {output_folder}")

# Variable to track the number of frames captured
frame_count = 0

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Check if the frame is read successfully
    if not ret:
        print("Error: Couldn't read frame.")
        break

    # Display the frame
    cv2.imshow("USB Camera", frame)

    # Check for key press (27 is the ASCII code for the 'Esc' key)
    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        print("Exiting...")
        break
    elif key == ord('c'):
        # Save the frame when the 'c' key is pressed
        frame_count += 1
        file_path = os.path.join(output_folder, f"captured_frame_{frame_count}.jpg")
        cv2.imwrite(file_path, frame)
        print(f"Frame {frame_count} captured and saved to: {file_path}")

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()

