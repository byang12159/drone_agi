import cv2
import os
import numpy as np
import pickle

img_usb = cv2.imread("./captured_frame_1_undistort.jpg")
img_usb = cv2.rotate(img_usb, cv2.ROTATE_180)
# cv2.imshow("undistored",img_usb)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

img_realsense = cv2.imread("./realsense_color.jpg")
# cv2.imshow("undistored",img_realsense)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

images = np.hstack((img_usb, img_realsense))
cv2.imshow("undistored",images)
cv2.waitKey(0)
cv2.destroyAllWindows()



with open('./picam/picam_params_pickle.p', 'rb') as file:
    # Load the first object
    data = pickle.load(file)

# Now 'data1' and 'data2' contain the deserialized objects
print(data)
usb_mtx = data['intri_mtx'] 
usb_dist = data['dist']
usb_extri = data['extri_mtx']