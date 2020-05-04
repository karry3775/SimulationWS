import numpy as np
import cv2
import matplotlib.pyplot as plt

img_left = cv2.imread('../images/left_color.png',0)
img_right = cv2.imread('../images/right_color.png',0)
# first step is to detect Fast features on the full image

# lets preprocess image
img_left = img_left[100:300, :]
img_right = img_right[100:300, :]
# lets blur the image
img_left = cv2.GaussianBlur(img_left, (5,5), 0)
img_right = cv2.GaussianBlur(img_right, (5,5), 0)


def returnKpImage(img, fast, c):
    kp = fast.detect(img, None)
    img_new = cv2.drawKeypoints(img_left, kp, None, color = c)
    return img_new, kp

# Initiate the FAST object with default values
fast = cv2.FastFeatureDetector_create()
# print all the default params
print("Threshold: {}".format(fast.getThreshold()))
print("nonmaxSuppression: {}".format(fast.getNonmaxSuppression()))
print("neighborhood: {}".format(fast.getType()))
# disable nonmaxSuppression
fast.setNonmaxSuppression(False);

img_left_kp, kp_left = returnKpImage(img_left, fast, (0,255,0))
img_right_kp, kp_right = returnKpImage(img_right, fast, (0,0,255))

cv2.imshow("left image", img_left_kp)
combined_image = cv2.drawKeypoints(img_left_kp, kp_right, None, color = (255,0,0))

cv2.imshow("combined_image", combined_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
