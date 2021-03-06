import pickle
import robot_sim.robots.ur3_dual.ur3_dual as ur3d
import rbt_con.force_control as ur3dx
import visualization.panda.world as wd
import modeling.geometric_model as gm
import modeling.collision_model as cm
import numpy as np
import cv2
import img_to_depth as itd
import time

video1 = cv2.VideoCapture(1)
itd_cvter = itd.ImageToDepth(0)
pixmm = itd_cvter.pix_to_mm

def hm2pos(hm):
    depth_max = np.max(hm)
    hm_map = hm/depth_max * 255
    border = 20
    border2 = 90

    width, height = np.shape(hm_map)[:]
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
    p = 15
    fshift[crow - p:crow + p, ccol - p:ccol + p] = 0
    f_ishift = np.fft.ifftshift(fshift)
    img_back = np.fft.ifft2(f_ishift)
    img_back = np.abs(img_back)
    img_back = (img_back / np.amax(img_back) * 255).astype("uint8")
    # cv2.imwrite("222.jpg", img_back)

    # apply canny edge detection
    edges = cv2.Canny(img_back, 250, 500)
    # print(np.shape(edges))
    edges = edges[border:height-border, border2:width]
    # print(np.shape(edges))

    # cv2.imshow(".",edges)
    # cv2.waitKey(50)

    # get hough lines
    result = img.copy()
    # print(np.shape(result))
    result = result[border:height-border, border2:width]
    # result = edges
    # print(np.shape(result))
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
        # print(x1,y1,x2,y2)
    if y1-y2 == 0:
        return None, None

    cv2.imshow("tst", result)
    cv2.waitKey(50)
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


inner_rad = 75/2
outer_rad = 99/2

center_rad = (inner_rad+outer_rad)/2

ur_dual_x = ur3dx.UR3DualX(lft_robot_ip='10.2.0.50', rgt_robot_ip='10.2.0.51', pc_ip='10.2.0.100')


def motion_stop(cn_flag, jnt = None, index = 0, path = None):
    time.sleep(0.5)
    while ur_dual_x.lft_arm_hnd.arm.is_program_running():
        pass
    while ur_dual_x.rgt_arm_hnd.arm.is_program_running():
        pass

    if cn_flag == 1:
        cn = "lft_arm"
    elif cn_flag == 2:
        cn = "rgt_arm"
    if index == 1:
        ur_dual_x.force_mode_move_lft(path)
    else:
        ur_dual_x.move_jnts(cn, jnt)

base = wd.World(cam_pos=[2, 1, 3], lookat_pos=[0, 0, 1.1])
gm.gen_frame().attach_to(base)
robot_s = ur3d.UR3Dual()

lft_jnt = np.array([1.0276005268096924, -2.85490066209902, -1.4274213949786585, 5.163119316101074, 4.332594871520996, 1.99058688])
# take the tape
print("-------------------move to initial position---------------------")
motion_stop(1, lft_jnt)
robot_s.fk("lft_arm", lft_jnt)
# detect
lft_pos, lft_rot = robot_s.get_gl_tcp("lft_arm")
new_jnt = lft_jnt.copy()
new_jnt[5] = new_jnt[5]+30/180*np.pi

time.sleep(0.5)
while ur_dual_x.lft_arm_hnd.arm.is_program_running():
    pass

ur_dual_x.lft_arm_hnd.open_gripper(speedpercentange=20, forcepercentage=0, fingerdistance=50)  # gripper control
while ur_dual_x.lft_arm_hnd.arm.is_program_running():
    pass
time.sleep(0.5)
flag = 0  # 1 if edge is detected, 0 if no edge is detected.
while (True):
    print("--------------detection start----------------")
    ur_dual_x.lft_arm_hnd.close_gripper(speedpercentange=20, forcepercentage=80)  # gripper control
    while ur_dual_x.lft_arm_hnd.arm.is_program_running():
        pass
    time.sleep(0.5)
    dur = 0
    theta = 0
    # time.sleep(10)
    count = 0
    tic = time.time()
    while (dur < 2.5):
        return_value, image = video1.read()
        depth, hm = itd_cvter.convert(image)
        dz, theta1 = hm2pos(hm)
        print(dz, theta1)
        if theta1 is not None:
            if np.abs(theta1 - theta) < 3 / 180 * np.pi or abs(np.abs(theta1 - theta) - np.pi) < 3 / 180 * np.pi:
                count = count + 1
            else:
                count = 0
            theta = theta1
            if count == 3:
                flag = 1
                print("The edge is detected.")
                break
        dur = time.time() - tic
    if flag == 1:
        print(theta)
        break

    time.sleep(0.5)
    while ur_dual_x.lft_arm_hnd.arm.is_program_running():
        pass
    ur_dual_x.lft_arm_hnd.move_jnts(new_jnt)

    time.sleep(0.5)
    while ur_dual_x.lft_arm_hnd.arm.is_program_running():
        pass
    ur_dual_x.lft_arm_hnd.open_gripper(speedpercentange=20, forcepercentage=0, fingerdistance=50)  # gripper control

    time.sleep(0.5)
    while ur_dual_x.lft_arm_hnd.arm.is_program_running():
        pass
    ur_dual_x.lft_arm_hnd.move_jnts(lft_jnt)

    print("----------------------")

time.sleep(0.5)
while ur_dual_x.lft_arm_hnd.arm.is_program_running():
    pass
ur_dual_x.lft_arm_hnd.open_gripper(speedpercentange=20, forcepercentage=0, fingerdistance=50)  # gripper control

# take the nail
lft_jnt_2 = np.array([0.8440605998039246, -3.249305073414938, -1.4211929480182093, 5.6194915771484375, 4.221709728240967, 8.435114685689108-2*np.pi])
motion_stop(1, lft_jnt_2)
lft_jnt_3 = np.array([0.39269161224365234, -2.1943357626544397, -2.079819981251852, 5.498745441436768, 4.005743980407715, 8.916003052388326-2*np.pi])
lft_jnt_4 = np.array([0.24308764934539795, -2.4585965315448206, -2.055237118397848, 5.86696720123291, 3.9646644592285156, 9.109419171010153-2*np.pi])
lft_jnt_5 = np.array([ 0.18426833, -2.04052908, -1.5584305,   5.65757132,  2.34814763,  3.46470356])
motion_stop(1, lft_jnt_4)
motion_stop(1, lft_jnt_3)
time.sleep(0.5)
while ur_dual_x.lft_arm_hnd.arm.is_program_running():
    pass
time.sleep(0.5)
ur_dual_x.lft_arm_hnd.close_gripper(speedpercentange=20, forcepercentage=80)  # gripper control
motion_stop(1, lft_jnt_4)

# hold the tape
rgt_jnt_1 = np.array([-1.01573354402651, -0.024298016224996388, 0.7465181350708008, -1.604677979146139, 1.9452768564224243, -3.5117350260363978])
rgt_jnt_2 = np.array([-0.8750742117511194, -0.06539041200746709, 1.0466761589050293, -1.6863907019244593, 2.029642105102539, -3.445847813283102])
rgt_jnt_3 = np.array([-1.00506002, -0.38177568,  0.76527023,  0.12596583,  1.29511535, -2.77655346])
motion_stop(2, rgt_jnt_1)
time.sleep(0.5)
while ur_dual_x.rgt_arm_hnd.arm.is_program_running():
    pass
ur_dual_x.rgt_arm_hnd.close_gripper(speedpercentange=100, forcepercentage=80)  # gripper control
time.sleep(2)
while ur_dual_x.rgt_arm_hnd.arm.is_program_running():
    pass
motion_stop(2, rgt_jnt_2)
motion_stop(2, rgt_jnt_3)
motion_stop(1, lft_jnt_5)

# peeling with force control
robot_s.fk("lft_arm", lft_jnt_5)
ini_pos, ini_rot = robot_s.get_gl_tcp("lft_arm")
pos_endlft = ini_pos + np.dot(ini_rot, np.array([0,0,0.02]))
pos_waypoint = ini_pos + np.dot(ini_rot, np.array([0,-0.02,0.01]))
lft_wayjnt = robot_s.ik("lft_arm", pos_waypoint, ini_rot, max_niter=1000, seed_jnt_values=np.array(lft_jnt_5))
lft_endjnt = robot_s.ik("lft_arm", pos_endlft, ini_rot, max_niter=1000, seed_jnt_values=np.array(lft_jnt_5))
print(lft_endjnt)
print(lft_wayjnt)

lft_jnt_6 = np.array([ 0.21818256, -2.07567245, -1.55833465,  5.72566223,  2.33887386,  3.46465564])
time.sleep(2)
motion_stop(1, lft_jnt_6)