import modeling.geometricmodel as gm
import visualization.panda.world as wd
import cv2
import img_to_depth as itd
import time
from classdef import Lookuptable
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from panda3d.core import NodePath
import visualization.panda.world as world
from modeling.geometricmodel import gen_pointcloud

# cam = cv2.VideoCapture(0)
# while (True):
#     return_value, image = cam.read()

image = cv2.imread("cam2/tst4.jpg")
frame = image
itd_cvter = itd.ImageToDepth()
pixmm = itd_cvter.pix_to_mm

depth, hm = itd_cvter.convert(frame)

depth_max = np.max(hm)
hm_map = hm / depth_max * 255
hm_map = hm_map.astype('uint8')
img = hm_map
f = np.fft.fft2(img)
fshift = np.fft.fftshift(f)
magnitude_spectrum = 100 * np.log(np.abs(fshift))
rows, cols = img.shape
crow, ccol = int(rows / 2), int(cols / 2)
p = 30
fshift[crow - p:crow + p, ccol - p:ccol + p] = 0
f_ishift = np.fft.ifftshift(fshift)
img_back = np.fft.ifft2(f_ishift)
img_back = np.abs(img_back)
img_back = (img_back / np.amax(img_back) * 255).astype("uint8")

# cv2.imshow("tst", img_back)

# # img_back = cv2.cvtColor(img_back,cv2.COLOR_BGR2GRAY)
# gray = 255 - img_back
#
# # threshold
# thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 13, 4)
# thresh = 255 - thresh
#
# # apply close to connect the white areas
# kernel = np.ones((3,3), np.uint8)
# morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
# kernel = np.ones((1,9), np.uint8)
# morph = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, kernel)

# apply canny edge detection
edges = cv2.Canny(img_back, 150, 200)

# get hough lines
result = img.copy()
lines = cv2.HoughLines(edges, 1, np.pi / 180, 80, min_theta=1)
# print(lines)
# Draw line on the image
result = cv2.cvtColor(result, cv2.COLOR_GRAY2RGB)
print(cols, rows)
centerx = cols/2
centery = rows/2
for rho, theta in lines[0]:
    a = np.cos(theta)
    b = np.sin(theta)
    # x0 = a * rho
    # y0 = b * rho
    # x1 = int(x0 + 1000 * (-b))
    # y1 = int(y0 + 1000 * (a))
    # x2 = int(x0 - 1000 * (-b))
    # y2 = int(y0 - 1000 * (a))
    # cv2.line(result, (x1, y1), (x2, y2), (0, 0, 255), 1)

x0 = int(centerx)
y0 = int(rho/b-x0/np.tan(theta))    # center of line
# print(x0,y0)
cv2.circle(result, (x0, y0), 50, (0,0,255), 1)

radius = min(rho, 10)
mask = np.zeros((rows, cols))
xq, yq = np.meshgrid(np.arange(rows), np.arange(cols))
xq = xq - x0
yq = yq - y0
rq = xq*xq+yq*yq
wq = yq + xq/np.tan(theta)

mask=(rq<radius*radius).astype('uint8')
mask1 = (wq>rho/np.sin(theta)).astype('uint8')
mask2 = (wq<rho/np.sin(theta)).astype('uint8')

mask1 = mask*mask1
mask2 = mask*mask2
sum_up = np.sum(mask1*hm, 1)
sum_low = np.sum(mask2*hm, 1)

dz = (y0 - centery)*pixmm



