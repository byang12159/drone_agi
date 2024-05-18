import cv2
import numpy as np

point_real = np.array([[200,300,1]], dtype=np.float32).T

img = cv2.imread("./checkerboard_D435i/captured_frame_1.jpg")
circleimg = cv2.circle(img, (200,300), 5,(0,255,0),2)
# cv2.imshow("realsense",circleimg)
# cv2.waitKey(0)
# cv2.destroyAllwindows()

mtx1 = np.array([[604.735,   0.,    318.968],
 [  0.   , 604.735 ,244.451],
 [  0.,0.,  1.  ]])

D1 = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])

R = np.array([[ 0.69970652, -0.04754301,  0.71284672],
 [ 0.29618189 , 0.92730362 ,-0.22887613],
 [-0.65014389  ,0.37127841  ,0.66292177]])
T = np.array([[-11.08921313],
 [ -1.48844197],
 [  6.795194  ]])
mtx2 = np.array([[487.99835295  , 0.  ,       317.56103286],
 [  0.     ,    488.94034535 ,264.75575731],
 [  0.      ,     0.        ,   1.        ]]
)

D2 = np.array([[-0.45875652,  0.56019265, -0.00399057 , 0.00325751, -0.77486947]])
# print(np.hstack((np.eye(3),np.zeros((3,1)))))
projection1 = np.dot(mtx1, np.hstack((np.eye(3),np.zeros((3,1)))))
projection2 = np.dot(mtx2, np.hstack((R,T)))

point_world = np.dot(np.linalg.inv(projection1), point_real)
point_cam2_homo = np.dot(projection2, point_world)
point_cam2_pixel = point_cam2_homo[:2]/ point_cam2_homo[2]

print("origiinal point realsense", point_real[:2].T)
print("transofrmade point", point_cam2_pixel)