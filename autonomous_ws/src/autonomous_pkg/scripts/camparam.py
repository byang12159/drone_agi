import cv2

def get_camera_property_range(camera, property_id):
    # Attempt to get min, max, and default values for a given property
    camera.set(cv2.CAP_PROP_EXPOSURE, 100)
    camera.set(cv2.CAP_PROP_CONTRAST, 64)

    min_val = camera.get(cv2.CAP_PROP_AUTO_EXPOSURE)
    max_val = camera.get(cv2.CAP_PROP_AUTO_EXPOSURE)
    default_val = camera.get(property_id)
    return min_val, max_val, default_val

# Open a connection to the USB camera
camera = cv2.VideoCapture(0)  # Change the index if you have multiple cameras

if not camera.isOpened():
    print("Error: Could not open camera.")
    exit()

# Query the exposure range
exposure_min, exposure_max, exposure_default = get_camera_property_range(camera, cv2.CAP_PROP_EXPOSURE)
print(f"Exposure range: Min={exposure_min}, Max={exposure_max}, Default={exposure_default}")

# Query the contrast range
contrast_min, contrast_max, contrast_default = get_camera_property_range(camera, cv2.CAP_PROP_CONTRAST)
print(f"Contrast range: Min={contrast_min}, Max={contrast_max}, Default={contrast_default}")

# Release the camera
camera.release()
