import numpy as np
import cv2
import img_to_depth as itd
import time

img = cv2.imread("cam2/tape_1.jpg")
itd_cvter = itd.ImageToDepth(0)
pixmm = itd_cvter.pix_to_mm
_, hm = itd_cvter.convert(img)

def hm2pos(hm):
    depth_max = np.max(hm)
    hm_map = hm/depth_max * 255

    width, height = np.shape(hm_map)[:]
    hm_map = hm_map.astype('uint8')
    img = hm_map
    f = np.fft.fft2(img)
    fshift = np.fft.fftshift(f)
    magnitude_spectrum = 100 * np.log(np.abs(fshift))
    rows, cols = img.shape
    crow, ccol = int(rows / 2), int(cols / 2)
    p = 100
    fshift[crow - p:crow + p, ccol - p:ccol + p] = 0
    f_ishift = np.fft.ifftshift(fshift)
    img_back = np.fft.ifft2(f_ishift)
    img_back = np.abs(img_back)
    img_back = (img_back / np.amax(img_back) * 255).astype("uint8")
    cv2.imshow(".",img_back)
    cv2.waitKey(0)
    # cv2.imwrite("222.jpg", img_back)

    # apply canny edge detection
    edges = cv2.Canny(img_back, 250, 500)
    cv2.imshow(".",edges)
    cv2.waitKey(0)
    # get hough lines
    result = img.copy()
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 20, minLineLength=1)
    print(lines)
    # Draw line on the image
    result = cv2.cvtColor(result, cv2.COLOR_GRAY2RGB)
    # print(cols, rows)
    centerx = cols / 2
    centery = rows / 2
    if lines is None:
        return None, None
    for rho, theta in lines[0]:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(result, (x1, y1), (x2, y2), (0, 0, 255), 1)
        # print(x1,y1,x2,y2)
    if y1-y2 == 0:
        return None, None

    cv2.imshow("tst", result)
    cv2.waitKey(0)
    # x0 = int(centerx)
    # y0 = int(rho / b - x0 / np.tan(theta))  # center of line
    # # print(x0,y0)
    # cv2.circle(result, (x0, y0), 50, (0, 0, 255), 1)
    #
    # radius = min(rho, 10)
    if abs(rho)<10:
        return None, None
    # mask = np.zeros((rows, cols))
    # xq, yq = np.meshgrid(np.arange(cols), np.arange(rows))
    # xq = xq - x0
    # yq = yq - y0
    # rq = xq * xq + yq * yq
    # wq = yq + xq / np.tan(theta)
    #
    # mask = (rq < radius * radius).astype('uint8')
    # mask1 = (wq > rho / np.sin(theta)).astype('uint8')
    # mask2 = (wq < rho / np.sin(theta)).astype('uint8')
    #
    # mask1 = mask * mask1
    # mask2 = mask * mask2
    # # sum_up = np.sum(mask1 * hm, 1)
    # # sum_low = np.sum(mask2 * hm, 1)
    #
    # dz = (y0 - centery) * pixmm
    if (theta > np.pi/4 and theta < np.pi/4*3) or (theta>5/4*np.pi and theta<7/4*np.pi):
        return None, None
    dz = 0
    return dz, theta

dz, theta = hm2pos(hm)