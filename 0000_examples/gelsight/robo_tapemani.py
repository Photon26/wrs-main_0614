import cv2
import img_to_depth as itd
import time
import numpy as np
import math
import visualization.panda.world as wd
import modeling.geometricmodel as gm
import modeling.collisionmodel as cm
import robotsim.robots.ur3_dual.ur3_dual as ur3d
import motion.probabilistic.rrt_connect as rrtc
import robotcon.ur.ur3_dual_x as ur3dx
import pickle

video1 = cv2.VideoCapture(0)
itd_cvter = itd.ImageToDepth()
pixmm = itd_cvter.pix_to_mm

def hm2pos(hm):
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

    # apply canny edge detection
    edges = cv2.Canny(img_back, 150, 200)

    # get hough lines
    result = img.copy()
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 100, min_theta=1)
    # print(lines)
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

    cv2.imshow("tst", result)
    cv2.waitKey(50)
    x0 = int(centerx)
    y0 = int(rho / b - x0 / np.tan(theta))  # center of line
    # print(x0,y0)
    cv2.circle(result, (x0, y0), 50, (0, 0, 255), 1)

    radius = min(rho, 10)
    mask = np.zeros((rows, cols))
    xq, yq = np.meshgrid(np.arange(cols), np.arange(rows))
    xq = xq - x0
    yq = yq - y0
    rq = xq * xq + yq * yq
    wq = yq + xq / np.tan(theta)

    mask = (rq < radius * radius).astype('uint8')
    mask1 = (wq > rho / np.sin(theta)).astype('uint8')
    mask2 = (wq < rho / np.sin(theta)).astype('uint8')

    mask1 = mask * mask1
    mask2 = mask * mask2
    sum_up = np.sum(mask1 * hm, 1)
    sum_low = np.sum(mask2 * hm, 1)

    dz = (y0 - centery) * pixmm
    return dz, theta

# base = wd.World(cam_pos=[2, 1, 3], lookat_pos=[0, 0, 1.1])
# gm.gen_frame().attach_to(base)
# robot_s
# component_name = 'lft_arm'
robot_instance = ur3d.UR3Dual()
ur_dual_x = ur3dx.UR3DualX(lft_robot_ip='10.2.0.50', rgt_robot_ip='10.2.0.51', pc_ip='10.2.0.100')
# robot_meshmodel = robot_instance.gen_meshmodel(toggle_tcpcs=True)
# robot_meshmodel.attach_to(base)

rgtjnt = pickle.load(open("pose_rgt.pkl", "rb"))
ur_dual_x.rgt_arm_hnd.move_jnts(rgtjnt)

lftjnts = pickle.load((open("pose_lft.pkl", "rb")))
ini_pose = np.array(lftjnts[0])
ur_dual_x.lft_arm_hnd.move_jnts(ini_pose)
ur_dual_x.lft_arm_hnd.move_jntspace_path(lftjnts,interval_time=0.7)

theta = 0
# time.sleep(10)
count = 0
while True:
    return_value, image = video1.read()
    depth, hm = itd_cvter.convert(image)
    dz, theta1 = hm2pos(hm)
    print(theta1)
    if theta1 is not None:
        if np.abs(theta1-theta) < 3/180*np.pi:
            count = count+1
        else:
            count = 0
        theta = theta1
        if count == 10:
            break


# init_lft_arm_jnt_values = robot_s.lft_arm.get_jnt_values()
# init_rgt_arm_jnt_values = robot_s.rgt_arm.get_jnt_values()
# full_jnt_values = np.hstack((init_lft_arm_jnt_values, init_rgt_arm_jnt_values))

# full_jnt_values = np.hstack((robot_instance.lft_arm.homeconf, robot_instance.rgt_arm.homeconf))
# goal_lft_arm_jnt_values = np.array([0, -math.pi / 2, -math.pi/3, -math.pi / 2, math.pi / 6, math.pi / 6])
# goal_rgt_arm_jnt_values = np.array([0, -math.pi/4, 0, math.pi/2, math.pi/2, math.pi / 6])
# robot_instance.fk(component_name="lft_arm", jnt_values = np.array([0, -math.pi / 2, -math.pi/3, -math.pi / 2, math.pi / 6, math.pi / 6]))
# pos = robot_instance.lft_arm.get_gl_tcp()


# gm.gen_sphere(pos[0]).attach_to(base)
#


# pose = []
# count = 0
# print("start")
# while count < 100:
#     tmp_pos = ur_dual_x.lft_arm_hnd.get_jnt_values()
#     print(count)
#     if len(pose)>1:
#         if np.linalg.norm(np.array(pose[-1])-np.array(tmp_pos)) < 0.01/180*np.pi:
#             print(np.linalg.norm(np.array(pose[-1])-np.array(tmp_pos)))
#             count = count+1
#             time.sleep(0.01)
#             continue
#     pose.append(tmp_pos)
#     time.sleep(0.1)
#
# print("done")
# pickle.dump(pose, open("pose_lft.pkl", "wb"))
# tmp_pos = ur_dual_x.lft_arm_hnd.get_jnt_values()
# pickle.dump(tmp_pos, open("pose_rgt.pkl", "wb"))
# # # base.run()

pose = pickle.load(open("pose_lft.pkl","rb"))
ini_pose = np.array(pose[0])
# ur_dual_x.lft_arm_hnd.move_jnts(ini_pose)
# ur_dual_x.lft_arm_hnd.move_jntspace_path(pose,interval_time=0.7)

# pose = ur_dual_x.lft_arm_hnd.get_jnt_values()
robot_instance.fk(component_name="lft_arm", jnt_values=ini_pose)


inijnt = ur_dual_x.lft_arm_hnd.get_jnt_values()
robot_instance.lft_arm.fk(inijnt)
pose_hnd = robot_instance.get_gl_tcp(manipulator_name="lft_arm")
pos_tape = pose_hnd[0]+np.dot(pose_hnd[1], np.array([0,-0.01,0.01875+dz/1000]))
pos_hnd_new = pos_tape+np.dot(pose_hnd[1], np.array([0,0.01,-0.0405]))
pos_hnd_move_1 = pos_hnd_new+np.dot(pose_hnd[1], np.array([0,0,0.003]))
pos_hnd_move_2 = pos_hnd_new+np.dot(pose_hnd[1], np.array([0,0,-0.003]))
newjnt_1 = robot_instance.ik("lft_arm", pos_hnd_move_1, pose_hnd[1], seed_jnt_values= inijnt)
newjnt_2 = robot_instance.ik("lft_arm", pos_hnd_move_2, pose_hnd[1], seed_jnt_values= inijnt)
ur_dual_x.lft_arm_hnd.move_jnts((newjnt_1))
ur_dual_x.lft_arm_hnd.move_jnts((newjnt_2))


# base.run()

