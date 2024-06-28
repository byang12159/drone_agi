import numpy as np
import pickle
import cv2

with open('./picam/picam_params_pickle.p', 'rb') as file:
    # Load the first object
    data = pickle.load(file)


# Now 'data1' and 'data2' contain the deserialized objects
print(data)
mtx = data['intri_mtx'] 
dist = data['dist']

img = cv2.imread("./captured_frame_1.jpg")

dst = cv2.undistort(img, mtx, dist, None, mtx)

cv2.imshow("undistored",dst)
cv2.waitKey(0)
cv2.destroyAllWindows()

cv2.imwrite("captured_frame_1_undistort.jpg",dst)