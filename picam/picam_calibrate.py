import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import glob
import cv2
import pickle


def Calibrate():
    # Number of object points
    num_intersections_in_x = 4
    num_intersections_in_y = 6
    
    # Size of square in meters
    square_size = 0.0375
    
    # Arrays to store 3D points and 2D image points
    obj_points = []
    img_points = []
    
    # Prepare expected object 3D object points (0,0,0), (1,0,0) ...
    object_points = np.zeros((num_intersections_in_x *num_intersections_in_y,3), np.float32)
    object_points[:,:2] = np.mgrid[0:num_intersections_in_x, 0:num_intersections_in_y].T.reshape(-1,2)
    object_points = object_points*square_size
    
    fnames = glob.glob('./captured_frames_raw_data/captured_frame_'+'*.'+'jpg')
    
    print(len(fnames) , "number of images found")
    for fname in fnames:
        img = cv2.imread(fname)
        img= cv2.flip(img,-1)
        # cv2.namedWindow("main", cv2.WINDOW_NORMAL)
        # cv2.imshow("main", img)
        # cv2.waitKey(0)
        img_size = (img.shape[1], img.shape[0])
        gray_scale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)     

        # Find chess board corners
        ret, corners = cv2.findChessboardCorners(gray_scale, (num_intersections_in_x, num_intersections_in_y), None)
        if ret:
            obj_points.append(object_points)
            img_points.append(corners)
            
            # Draw the corners
            drawn_img = cv2.drawChessboardCorners(img, (7,7), corners, ret)
            # Uncomment the below code (43-47) if you want to visualize the detected corners
            drawn_img = cv2.resize(drawn_img, (500,500))
            cv2.namedWindow("main", cv2.WINDOW_NORMAL)
            cv2.imshow("main", drawn_img)
            cv2.waitKey(0)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, img_size, None, None)

    img = cv2.imread(fnames[19])
    dst = cv2.undistort(img, mtx, dist, None, mtx)
    f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
    ax1.imshow(img)
    ax1.set_title('Original Image', fontsize=15)
    ax2.imshow(dst)
    ax2.set_title('Undistorted Image', fontsize=15)
    plt.show()

    return [ret, mtx, dist, rvecs, tvecs]



if __name__ == "__main__":
    ret, mtx, dist, rvecs, tvecs = Calibrate()
    print("ret",ret)
    print("camera intrinsic matrix: \n",mtx)
    print("distortion coefficients: \n",dist)

    rvec, tvec = rvecs[0], tvecs[0]
    extrinsic_mtx = np.hstack((cv2.Rodrigues(rvec)[0] ,tvec))
    print("extrinsic matrix: \n",extrinsic_mtx)
    #print("tvecs",tvecs)
    # Code used to store varaibles, Inntrinsic params as mtx, and distortion coeficient as dist. Uncomment the below lines to store them again.
    dist_pickle = {}
    dist_pickle["intri_mtx"] = mtx
    dist_pickle["extri_mtx"] = extrinsic_mtx
    dist_pickle["dist"] = dist
    pickle.dump(dist_pickle, open("picam_params_pickle.p", "wb"))

