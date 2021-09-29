import cv2
import numpy as np

cap = cv2.VideoCapture(2)
while(True):
    ret, frame = cap.read()
    print(np.shape(frame))
    cv2.imshow("tst", frame)
    cv2.waitKey(1)


# import numpy as np
# import cv2
#
# cap = cv2.VideoCapture(0)
# cap1 = cv2.VideoCapture(1)
# # Define the codec and create VideoWriter object
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# fourcc2 = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))
# out2 = cv2.VideoWriter('output2.avi', fourcc2, 20.0, (640, 480))
#
# while(cap.isOpened() and cap1.isOpened()):
#     ret, frame = cap.read()
#     ret2, frame2 = cap1.read()
#     if ret == True and ret2 == True:
#         # frame = cv2.flip(frame,0)
#
#         # write the flipped frame
#         out.write(frame)
#         out2.write(frame2)
#
#         cv2.imshow('frame',frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
#     else:
#         break
#
# # Release everything if job is finished
# cap.release()
# cap1.release()
# out.release()
# out2.release()
# cv2.destroyAllWindows()
#
# from calibrate_gelsight import takeimg
# import cv2
#
# takeimg("cam3", 0, 0, "pixmm")

