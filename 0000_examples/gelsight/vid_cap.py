import cv2
import numpy as np
import img_to_depth
import os

cap1 = cv2.VideoCapture(1)
cap2 = cv2.VideoCapture(0)
# cap3 = cv2.VideoCapture(2)
itd_cvter1 = img_to_depth.ImageToDepth(0)
itd_cvter2 = img_to_depth.ImageToDepth(1)

fourcc1 = cv2.VideoWriter_fourcc('X', '2', '6', '4')
fourcc2 = cv2.VideoWriter_fourcc('X', '2', '6', '4')
# fourcc3 = cv2.VideoWriter_fourcc(*'XVID')

datafolder = "/data"
if not os.path.exists(datafolder):
    os.makedirs(datafolder)

datanum = 100
for i in range(0, datanum):
    outputfolder = datafolder+'/'+str(i)
    if not os.path.exists(outputfolder):
        os.makedirs(outputfolder)

    out1 = cv2.VideoWriter('cam2.mp4', fourcc1, 10.0, (484, 397), False)
    out2 = cv2.VideoWriter('cam3.mp4', fourcc2, 10.0, (448, 424), False)
    # out3 = cv2.VideoWriter('3.avi', fourcc2, 20.0, (640, 480))

    while(cap1.isOpened() and cap2.isOpened()):
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()
        # ret3, frame3 = cap3.read()
        if ret1 == True and ret2 == True:
            depth1, hm1 = itd_cvter1.convert(frame1)
            depth2, hm2 = itd_cvter2.convert(frame2)

            depth_max = np.max(hm1)
            hm_map1 = hm1 / depth_max * 255
            hm_map1 = hm_map1.astype('uint8')
            img = hm_map1
            print(np.shape(img))

            depth_max = np.max(hm2)
            print(depth_max)
            hm_map2 = hm2 / depth_max * 255
            hm_map2 = hm_map2.astype('uint8')
            img = hm_map2
            print(np.shape(img))


            cv2.imshow("tst", hm_map1)
            out1.write(hm_map1)
            out2.write(hm_map2)

            # out1.write(frame1)
            # out2.write(frame2)
            # out3.write(frame3)
            # cv2.imshow("tst2", hm_map2)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

    cap1.release()
    cap2.release()
    out1.release()
    # out2.release()
    cv2.destroyAllWindows()

