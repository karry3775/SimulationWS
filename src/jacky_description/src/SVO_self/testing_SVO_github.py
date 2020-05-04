import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

left_prev = cv2.imread('../images/left_color.png',0)
right_prev = cv2.imread('../images/right_color.png',0)

left_cur = cv2.imread('../images/left_color.png',0)
right_cur = cv2.imread('../images/right_color.png',0)

block = 15

# empirical values from P1, P2 as suggested in opencv documentation
P1 = 0 # block * block * 8
P2 = 0 # block * block *32

disparityEngine = cv2.StereoSGBM_create(minDisparity = 0, numDisparities = 16, blockSize = block, P1 = P1, P2 = P2)
# computing disparities for prev frame
prev_disparity = disparityEngine.compute(left_prev, right_prev).astype(np.float32)
prev_disparityA = np.divide(prev_disparity, 16.0)

# computing disparities for cur_frame
cur_disparity = disparityEngine.compute(left_cur, right_cur).astype(np.float32)
cur_disparityA = np.divide(cur_disparity, 16.0)

TILE_H = 10
TILE_W = 10

# create a fast feature engine
fastFeautureEngine = cv2.FastFeatureDetector_create()

# 20 x 10 (wxh) tiles for extracting less features from images
H, W = left_prev.shape
kp= []
idx = 0

for y in range(0, H, TILE_H):
    for x in range(0, W, TILE_W):
        imPatch = left_prev[y : y + TILE_H,  x : x + TILE_W]
        keypoints = fastFeautureEngine.detect(imPatch)
        for pt in keypoints:
            pt.pt = (pt.pt[0] + x, pt.pt[1] + y)

        if(len(keypoints) > 10):
            keypoints = sorted(keypoints, key = lambda x : -x.response)
            for kpt in keypoints[0:10]:
                kp.append(kpt)
        else:
            for kpt in keypoints:
                kp.append(kpt)


ftDebug = left_prev
ftDebug = cv2.drawKeypoints(left_prev, kp, ftDebug, color = (255, 0, 0))

# cv2.imshow("ftDebug bucket", ftDebug)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# pack keypoint 2-d coords into numpy array
trackPoints1 = np.zeros((len(kp), 1, 2), dtype = np.float32)
for i, kpt in enumerate(kp):
    trackPoints1[i, :, 0] = kpt.pt[0]
    trackPoints1[i, :, 1] = kpt.pt[1]


# Parameters for lucas kanade optical flow
lk_params = dict(winSize = (15, 15),
                 maxLevel = 2,
                 criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
# params for ShiTomasi corner detection
feature_params = dict(maxCorners = 100,
                      qualityLevel = 0.3,
                      minDistance = 7,
                      blockSize = 7)

trackPoints2, st, err = cv2.calcOpticalFlowPyrLK(left_prev, left_cur, trackPoints1, None, flags = cv2.MOTION_AFFINE, **lk_params)

# separate points that were tracked successfully
ptTrackable = np.where(st == 1, 1, 0).astype(bool)
trackPoints1_KLT = trackPoints1[ptTrackable, ...]
trackPoints2_KLT_t = trackPoints2[ptTrackable, ...]
trackPoints2_KLT = np.around(trackPoints2_KLT_t)


# among tracked points take points within error measure
error = 4
errTrackablePoints = err[ptTrackable, ...]
errThresholdPoints = np.where(errTrackablePoints < error, 1, 0).astype(bool)
trackPoints1_KLT = trackPoints1_KLT[errThresholdPoints, ...]
trackPoints2_KLT = trackPoints2_KLT[errThresholdPoints, ...]

#compute right image disparity displaced points
trackPoints1_KLT_L = trackPoints1_KLT
trackPoints2_KLT_L = trackPoints2_KLT

trackPoints1_KLT_R = np.copy(trackPoints1_KLT_L)
trackPoints2_KLT_R = np.copy(trackPoints2_KLT_L)
selectedPointMap = np.zeros(trackPoints1_KLT_L.shape[0])

disparityMinThres = 0.0
disparityMaxThres = 100.0

for i in range(trackPoints1_KLT_L.shape[0]):
    prevDisparity = prev_disparityA[int(trackPoints1_KLT_L[i,1]), int(trackPoints1_KLT_L[i,0])]
    curDisparity  = cur_disparityA[int(trackPoints2_KLT_L[i,1]), int(trackPoints2_KLT_L[i,0])]

    if (prevDisparity > disparityMinThres and prevDisparity < disparityMaxThres
        and curDisparity > disparityMinThres and curDisparity < disparityMaxThres):
        trackPoints1_KLT_R[i, 0] = trackPoints1_KLT_L[i, 0] - prevDisparity
        trackPoints2_KLT_R[i, 0] = trackPoints2_KLT_L[i, 0] - curDisparity
        selectedPointMap[i] = 1

selectedPointMap = selectedPointMap.astype(bool)
trackPoints1_KLT_L_3d = trackPoints1_KLT_L[selectedPointMap, ...]
trackPoints1_KLT_R_3d = trackPoints1_KLT_R[selectedPointMap, ...]
trackPoints2_KLT_L_3d = trackPoints2_KLT_L[selectedPointMap, ...]
trackPoints2_KLT_R_3d = trackPoints2_KLT_R[selectedPointMap, ...]

# 3d point cloud triangulation
numPoints = trackPoints1_KLT_L_3d.shape[0]
d3dPointsPrev = np.ones((numPoints,3))
d3dPointsCur = np.ones((numPoints,3))

b = 0.064
f = 238.3515418
cx = 200.5
cy = 200.5

def transformPoints(X, Y, Z):
    # convert to correct cordinate frame
    R = np.array([[0, 0 , 1],
                  [-1, 0, 0],
                  [0, -1, 0]])
    pt = np.array([[X],[Y],[Z]])
    pt = np.matmul(R, pt)
    return [float(pt[0]), float(pt[1]), float(pt[2])]

for i in range(numPoints):
    pLeft = trackPoints1_KLT_L_3d[i, :]
    pRight = trackPoints1_KLT_R_3d[i, :]
    # we need to basically get 3d points at this point
    xL, yL = pLeft
    xR, yR = pRight
    # find disparity
    d = xL - xR
    # let X, Y, Z cordinates
    Z = abs((f * b) / d)
    X = (Z * (xL - cx)) / f
    Y = (Z * (yL - cy)) / f
    # then we need to transform the points
    X, Y, Z  = transformPoints(X, Y, Z)
    d3dPointsPrev[i, 0] = X
    d3dPointsPrev[i, 1] = Y
    d3dPointsPrev[i, 2] = Z

# lets plot these points to better visualize
ax = plt.axes(projection='3d')
ax.scatter3D(d3dPointsPrev[:,0], d3dPointsPrev[:,1] , d3dPointsPrev[:,2], cmap='Greens');
plt.show()
