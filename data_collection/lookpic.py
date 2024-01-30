import cv2

path = "./realsense_color.jpg"

img = cv2.imread(path)
cv2.imshow("img",img)
cv2.waitKey(0)
cv2.destroyAllWindows()
