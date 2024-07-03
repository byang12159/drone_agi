import cv2
import cv2.aruco as aruco
import numpy as np

# Step 1: Create a custom dictionary with 1 marker
marker_size = 4  # 4x4 marker
custom_dict = aruco.custom_dictionary(1, marker_size)

# Step 2: Generate the marker and save it as an image
marker_id = 0  # ID of the marker
marker_image = np.zeros((marker_size * 100, marker_size * 100), dtype=np.uint8)
marker_image = aruco.drawMarker(custom_dict, marker_id, marker_size * 100, marker_image, 1)
cv2.imwrite("custom_marker_0.png", marker_image)

# Step 3: Detect and use the custom marker
# Define the parameters for the detector
parameters = aruco.DetectorParameters_create()

# Load an image containing the custom marker
image = cv2.imread('image_with_custom_marker.jpg')  # Replace with your image file

# Convert to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Detect markers
corners, ids, rejected = aruco.detectMarkers(gray, custom_dict, parameters=parameters)

# Draw detected markers on the image
image_with_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)

# Display the result
cv2.imshow('Detected Custom Marker', image_with_markers)
cv2.waitKey(0)
cv2.destroyAllWindows()
