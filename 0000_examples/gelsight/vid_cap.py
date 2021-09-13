import cv2
import numpy as np
import img_to_depth

cap1 = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture(1)
itd_cvter1 = img_to_depth.ImageToDepth(2)
itd_cvter2 = img_to_depth.ImageToDepth(3)

fourcc1 = cv2.VideoWriter_fourcc(*'XVID')
fourcc2 = cv2.VideoWriter_fourcc(*'XVID')
out1 = cv2.VideoWriter('1.avi', fourcc1, 20.0, (640, 480))
out2 = cv2.VideoWriter('2.avi', fourcc2, 20.0, (640, 480))

while(cap1.isOpened() and cap2.isOpened()):
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()
    if ret1 == True and ret2 == True:
        depth1, hm1 = itd_cvter1.convert(frame1)
        depth2, hm2 = itd_cvter2.convert(frame2)

        depth_max = np.max(hm1)
        hm_map1 = hm1 / depth_max * 255
        hm_map1 = hm_map1.astype('uint8')
        img = hm_map1
        depth_max = np.max(hm2)
        hm_map2 = hm2 / depth_max * 255
        hm_map2 = hm_map2.astype('uint8')
        img = hm_map2

        out1.write(hm_map1)
        out2.write(hm_map2)

        cv2.imshow("tst", hm_map1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

cap1.release()
cap2.release()
out1.release()
out2.release()
cv2.destroyAllWindows()

