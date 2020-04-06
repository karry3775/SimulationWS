import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cv2
import numpy as np
from matplotlib import pyplot as plt


def downsample_image(image, reduce_factor):
    for i in range(0, reduce_factor):
        # check if image is color or grayscale
        if len(image.shape) > 2:
            row, col = image.shape[:2]
        else:
            row, col = image.shape

        image = cv2.pyrDown(image, dstsize = (col//2, row//2))
    return image

#read images
img_left = cv2.imread("../images/left_color.png",0)
img_right = cv2.imread("../images/right_color.png",0)


# source : https://medium.com/@omar.ps16/stereo-3d-reconstruction-with-opencv-using-an-iphone-camera-part-iii-95460d3eddf0
# source : https://github.com/OmarPadierna/3DReconstruction/blob/master/Reconstruction/disparity.py
# set disparity map parameters
win_size = 5
min_disp = -1
max_disp = 63
num_disp = max_disp - min_disp # Needs to be divisible by 16

# create the Block matching object
stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
                               numDisparities = num_disp,
                               blockSize = 5,
                               uniquenessRatio = 5,
                               speckleWindowSize = 5,
                               speckleRange = 5,
                               disp12MaxDiff = 2,
                               P1 = 8*3*win_size**2,
                               P2 = 32*3*win_size**2)

stereo = cv2.StereoBM_create(numDisparities = 16, blockSize = 15)

# lets apply gaussian blur
# img_left = cv2.GaussianBlur(img_left, (3,3), 0)
# img_right = cv2.GaussianBlur(img_right, (3,3), 0)
# downsample images
# img_left = downsample_image(img_left, 1)
# img_right = downsample_image(img_right, 1)
# compute the disparity map
disp_map = stereo.compute(img_left, img_right).astype(np.float32) / 16.0
plt.imshow(disp_map, 'jet')



X = []
Y = []
Z = []
for i in range(len(disp_map)):
    for j in range(len(disp_map)):
        x = j; y = i; z = disp_map[i,j]
        X.append(z)
        Y.append(-x)
        Z.append(y)
# lets try plotting 3d representation of this
ax = plt.axes(projection='3d')
ax.scatter3D(X, Y, Z, c = Z, cmap='Greens');
plt.show()
# cv2.imshow("left_image", img_left)
# cv2.imshow("right_image", img_right)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
