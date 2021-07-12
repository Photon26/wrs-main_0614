import pickle
import robot_sim.robots.ur3_dual.ur3_dual as ur3d
import robot_con.ur.ur3_dual_x as ur3dx
import visualization.panda.world as wd
import modeling.geometric_model as gm
import modeling.collision_model as cm
import numpy as np
import cv2
import img_to_depth as itd
import time

video1 = cv2.VideoCapture(0)
itd_cvter = itd.ImageToDepth()
pixmm = itd_cvter.pix_to_mm

def hm2pos(hm):
    depth_max = np.max(hm)
    hm_map = hm / depth_max * 255
    border = 20

    height = np.shape(hm_map)[1]
    # hm_map[:border, :] = 0
    # hm_map[height-border: height, :] = 0
    # hm_map = hm_map[border:height-border, :]
    # cv2.imwrite("111.jpg", hm_map)

    hm_map = hm_map.astype('uint8')
    img = hm_map
    f = np.fft.fft2(img)
    fshift = np.fft.fftshift(f)
    magnitude_spectrum = 100 * np.log(np.abs(fshift))
    rows, cols = img.shape
    crow, ccol = int(rows / 2), int(cols / 2)
    p = 25
    fshift[crow - p:crow + p, ccol - p:ccol + p] = 0
    f_ishift = np.fft.ifftshift(fshift)
    img_back = np.fft.ifft2(f_ishift)
    img_back = np.abs(img_back)
    img_back = (img_back / np.amax(img_back) * 255).astype("uint8")
    # cv2.imwrite("222.jpg", img_back)

    # apply canny edge detection
    edges = cv2.Canny(img_back, 180, 200)
    print(np.shape(edges))
    edges = edges[border:height-border, :]
    print(np.shape(edges))

    # cv2.imshow(".",edges)
    # cv2.waitKey(50)

    # get hough lines
    result = img.copy()
    print(np.shape(result))
    result = result[border:height-border, :]
    # result = edges
    print(np.shape(result))
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 20)
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
        print(x1,y1,x2,y2)
    if x1-x2 == 0:
        return None, None

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
    # sum_up = np.sum(mask1 * hm, 1)
    # sum_low = np.sum(mask2 * hm, 1)

    dz = (y0 - centery) * pixmm
    if (theta > np.pi/4 and theta < np.pi/4*3) or (theta>5/4*np.pi and theta<7/4*np.pi):
        return None, None
    return dz, theta


inner_rad = 75/2
outer_rad = 99/2

center_rad = (inner_rad+outer_rad)/2

ur_dual_x = ur3dx.UR3DualX(lft_robot_ip='10.2.0.50', rgt_robot_ip='10.2.0.51', pc_ip='10.2.0.100')

base = wd.World(cam_pos=[2, 1, 3], lookat_pos=[0, 0, 1.1])
gm.gen_frame().attach_to(base)
robot_s = ur3d.UR3Dual()

pose_hnd = robot_s.get_gl_tcp(manipulator_name="lft_arm")
# print(pose_hnd)

ini_pos = np.array([0.3, 0.1,  1.4])
ini_rot_lft = np.array([[ 1, 0,  0],
       [ 0 , 0, -1],
       [ 0,  1,  0]])
# robot_meshmodel.attach_to(base)
center = ini_pos + np.dot(ini_rot_lft, np.array([0, -0.001*center_rad, 0]))

# object = cm.CollisionModel("tape_2side_210618.stl")
# object.set_pos(center)
# object.set_rgba([.5, .7, .3, 1])
# object.attach_to(base)

ini_rot_rgt = np.array([[ 1, 0,  0],
       [ 0 , 0, 1],
       [ 0,  -1,  0]])

jnt_list = []
pre_jnt = robot_s.ik("rgt_arm", ini_pos, ini_rot_rgt, max_niter=1000)
for theta in range(3,7):
    rotmat = np.array([[np.cos(np.pi / 15* theta + np.pi/4), 0, np.sin(np.pi / 15 * theta + np.pi/4)], [0, 1, 0],
                       [-np.sin(np.pi / 15 * theta + np.pi/4), 0, np.cos(np.pi / 15 * theta + np.pi/4)]])
    rot = np.dot(rotmat, ini_rot_rgt)
    pos = center + np.dot(rot, np.array([0, 0.001 * center_rad, 0]))
    newjnt_rgt = robot_s.ik("rgt_arm", pos, rot)
    if newjnt_rgt is None:
           newjnt_rgt = robot_s.ik("rgt_arm", pos, rot, pre_jnt, max_niter=1000)
    jnt_list.append(newjnt_rgt)
    pre_jnt = newjnt_rgt

# rgt hand hold the tape
# jnt_rgt = robot_s.ik("rgt_arm", ini_pos, ini_rot_rgt)

flag = 0  # 1 if edge is detected, 0 if no edge is detected.

while(True):
    ini_jnt_rgt = jnt_list[0]
    time.sleep(0.5)
    while ur_dual_x.rgt_arm_hnd.arm.is_program_running():
        pass
    ur_dual_x.rgt_arm_hnd.move_jnts(ini_jnt_rgt)
    time.sleep(0.5)
    while ur_dual_x.rgt_arm_hnd.arm.is_program_running():
        pass
    ur_dual_x.rgt_arm_hnd.close_gripper()

    # ini_pos_lft = ini_pos + np.dot(ini_rot_lft, np.array([0,0.005,0]))
    ini_pos_lft = ini_pos + np.dot(ini_rot_lft, np.array([0.005, 0, -0.045]))
    newjnt = robot_s.ik("lft_arm",ini_pos_lft, ini_rot_lft, max_niter=1000)
    robot_s.fk("lft_arm", newjnt)
    print(newjnt/3.14*180)
    ur_dual_x.lft_arm_hnd.move_jnts(newjnt)
    # robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=False)
    # robot_meshmodel.attach_to(base)

    #  rotate rgt hand
    for jnt in jnt_list:
        if jnt is not None:
            print("*")
            ur_dual_x.lft_arm_hnd.open_gripper(speedpercentange=20, forcepercentage=0, fingerdistance=50)  # gripper control
            time.sleep(0.8)
            ur_dual_x.rgt_arm_hnd.move_jnts(jnt)
            ur_dual_x.lft_arm_hnd.close_gripper(speedpercentange=20, forcepercentage=80)  # gripper control
            time.sleep(0.5)
            while ur_dual_x.lft_arm_hnd.arm.is_program_running():
                pass

            # time.sleep(0.5)
            robot_s.fk("rgt_arm", jnt)
            robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=False)
            robot_meshmodel.attach_to(base)

            #   detect the tape edge
            dur = 0
            theta = 0
            # time.sleep(10)
            count = 0
            tic = time.time()

            while (dur < 2.5):
                return_value, image = video1.read()
                depth, hm = itd_cvter.convert(image)
                dz, theta1 = hm2pos(hm)
                # print(theta1)
                if theta1 is not None:
                    if np.abs(theta1 - theta) < 3 / 180 * np.pi:
                        count = count + 1
                    else:
                        count = 0
                    theta = theta1
                    if count == 5:
                        flag = 1
                        # break
                dur = time.time() - tic
            if flag == 1:
                print(theta)
                break
        if flag == 1:
            break
            # pass

    if flag == 1:
        break
        # pass
    ur_dual_x.rgt_arm_hnd.open_gripper(fingerdistance=85)
    print("UUUUUU")
# base.run()
print(theta1)

inijnt = ur_dual_x.lft_arm_hnd.get_jnt_values()
robot_s.lft_arm.fk(inijnt)
pose_hnd = robot_s.get_gl_tcp(manipulator_name="lft_arm")
pos_tape = pose_hnd[0]+np.dot(pose_hnd[1], np.array([-0.02,-0.008,0]))
newjnt = robot_s.ik("lft_arm", pos_tape, pose_hnd[1], max_niter=10000)
ur_dual_x.lft_arm_hnd.open_gripper(fingerdistance=40)
time.sleep(0.5)
while ur_dual_x.lft_arm_hnd.arm.is_program_running():
    pass

ur_dual_x.lft_arm_hnd.move_jnts((newjnt))

inijnt = ur_dual_x.lft_arm_hnd.get_jnt_values()
robot_s.lft_arm.fk(inijnt)
pose_hnd = ur_dual_x.lft_arm_hnd.get_jnt_values()
pos_tape = pose_hnd[0]+np.dot(pose_hnd[1], np.array([0,0.01,0]))

rot_lft = np.array([[0,-1,0],[0,0,-1],[1,0,0]])
pos = pos_tape + np.dot(rot_lft, np.array([0.03, 0.006, -0.0257]))
newjnt = robot_s.ik("lft_arm", pos_tape, pose_hnd[1], max_niter=10000)
time.sleep(0.5)
while ur_dual_x.lft_arm_hnd.arm.is_program_running():
    pass

ur_dual_x.lft_arm_hnd.move_jnts((newjnt))