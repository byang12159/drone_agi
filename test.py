import cv2
# for i in range(5):  # Try up to 5 camera indices
#     cap = cv2.VideoCapture(i)
#     if cap.isOpened():
#         print(f"Camera {i} is available.")
#         cap.release()
#     else:
#         print(f"Camera {i} is not available.")

# Initialize the camera (0 is the default camera, change to 1 or higher if needed)
def default_cap():
    cap = cv2.VideoCapture(2)

    if not cap.isOpened():
        print("Error: Could not open video capture.")
    else:
        # Capture a single frame
        ret, frame = cap.read()

        if ret:
            # Display the captured image
            cv2.imshow('Captured Image', frame)

            # Save the captured image to a file
            cv2.imwrite('captured_image.jpg', frame)

            # Wait for a key press and close the window
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print("Error: Could not read the frame.")

    # Release the camera when done
    cap.release()

def get_camera():
    cap = cv2.VideoCapture(2)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 100) 
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)

    cv2.waitKey(500)
    return cap

def drone_cap():
    cap = get_camera()
    ret, frame = cap.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)

    # Display the captured image
    cv2.imshow('Captured Image', frame)

    # Save the captured image to a file
    cv2.imwrite('captured_image.jpg', frame)

    # Wait for a key press and close the window
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cap.release()

if __name__ == "__main__":
    drone_cap()