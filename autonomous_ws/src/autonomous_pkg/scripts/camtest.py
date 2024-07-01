import cv2
import time


def main():
    # Open the video capture
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    img_width = 640
    img_height = 480
    frame_rate = 100
    cap.set(2, img_width)
    cap.set(4, img_height)
    cap.set(5, frame_rate)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
   
    # Initialize variables to calculate FPS
    fps = 0
    frame_count = 0
    start_time = time.time()


    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image")
            break


        # Calculate FPS
        frame_count += 1
        end_time = time.time()
        elapsed_time = end_time - start_time
        if elapsed_time >= 1.0:
            fps = frame_count / elapsed_time
            start_time = time.time()
            frame_count = 0


        # # Display FPS on the frame
        # cv2.putText(frame, f'FPS: {fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        # # Display the frame
        # cv2.imshow('Camera', frame)
        print(fps)
        # Exit when 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    # Release the capture and close the windows
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
