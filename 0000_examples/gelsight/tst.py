import pickle
import robot_sim.robots.ur3_dual.ur3_dual as ur3d
import robot_con.ur.ur3_dual_x as ur3dx
import visualization.panda.world as wd
import modeling.geometric_model as gm
import numpy as np
import cv2
import img_to_depth as itd

# inner_rad = 75/2
# outer_rad = 99/2
#
# robot_instance = ur3d.UR3Dual()
# ur_dual_x = ur3dx.UR3DualX(lft_robot_ip='10.2.0.50', rgt_robot_ip='10.2.0.51', pc_ip='10.2.0.100')
#
#
#
# base = wd.World(cam_pos=[2, 1, 3], lookat_pos=[0, 0, 1.1])
# gm.gen_frame().attach_to(base)
# component_name = "lft_arm"
#
# jnt = ur_dual_x.lft_arm_hnd.get_jnt_values()
# robot_instance.fk(component_name,np.array(jnt))
# pose_hnd = robot_instance.get_gl_tcp(manipulator_name=component_name)
# print(pose_hnd)
# print(jnt)
# print("shi fou peng zhuang",robot_instance.is_collided())
# robot_meshmodel = robot_instance.gen_meshmodel(toggle_tcpcs=True)
# robot_meshmodel.attach_to(base)
#
# # pos = np.array([ 0.3, 0,  1.4])
# #
# # ini_rot_rgt = np.array([[ 1, 0,  0],
# #        [ 0 , 0, 1],
# #        [ 0,  -1,  0]])
# # ini_rot_lft = np.array([[ 1, 0,  0],
# #        [ 0 , 0, -1],
# #        [ 0,  1,  0]])
# # gm.gen_frame(pos=pose_hnd[0],rotmat=pose_hnd[1],length=0.4).attach_to(base)
# # print(robot_instance.ik("lft_arm",tgt_pos=pose_hnd[0],tgt_rotmat=pose_hnd[1],seed_jnt_values=jnt,max_niter=1000))
# base.run()
#
#
# # ini_rot_lft = np.array([[ 1, 0,  0],
# #        [ 0 , 0, -1],
# #        [ 0,  1,  0]])
# # ini_rot_rgt = np.array([[ 1, 0,  0],
# #        [ 0 , 0, 1],
# #        [ 0,  -1,  0]])
# # i = 0
# # # poslist = []
# # # while (i < 1):
# # #     j = 1.1
# # #     while j < 1.6:
# # #         pos = np.array([i, 0, j])
# # #
# # #         jnt = robot_instance.lft_arm.ik(pos, ini_rot_lft)
# # #         jnt2 = robot_instance.rgt_arm.ik(pos, ini_rot_rgt)
# # #         # if jnt is not None and jnt2 is not None:
# # #         if jnt2 is not None:
# # #             print(i,j)
# # #             poslist.append([i,j])
# # #         j = j+0.05
# # #     i = i+0.05
# # #
# # # print(poslist)
# #
# # base = wd.World(cam_pos=[2, 1, 3], lookat_pos=[0, 0, 1.1])
# # gm.gen_frame().attach_to(base)
# #
# # # jnt = ur_dual_x.lft_arm_hnd.get_jnt_values()
# # # robot_instance.fk("lft_arm",np.array(jnt))
# # # pose_hnd = robot_instance.get_gl_tcp(manipulator_name="lft_arm")
# # # print(pose_hnd)
# # # print(jnt)
# # # print("shi fou peng zhuang",robot_instance.is_collided())
# # pos = np.array([0,0,1.15])
# # jnt = robot_instance.rgt_arm.ik(pos, ini_rot_rgt)
# # robot_instance.fk("rgt_arm", jnt)
# # robot_meshmodel = robot_instance.gen_meshmodel(toggle_tcpcs=True)
# # robot_meshmodel.attach_to(base)
# # print(robot_instance.is_collided())
# # # ur_dual_x.move_jnts("rgt_arm", jnt)
# # # print(robot_instance.ik("lft_arm",tgt_pos=pose_hnd[0],tgt_rotmat=pose_hnd[1]))
# # base.run()

import calibrate_gelsight
import cv2
video1 = cv2.VideoCapture(0)
itd_cvter = itd.ImageToDepth()
pixmm = itd_cvter.pix_to_mm

# calibrate_gelsight.takeimg("cam2", 0, filename="t")

def hm2pos(hm):
    depth_max = np.max(hm)
    hm_map = hm / depth_max * 255
    border = 20

    height = np.shape(hm_map)[1]
    # hm_map[:border, :] = 0
    # hm_map[height-border: height, :] = 0
    # hm_map = hm_map[border:height-border, :]
    cv2.imwrite("111.jpg", hm_map)

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
    cv2.imwrite("222.jpg", img_back)

    # apply canny edge detection
    edges = cv2.Canny(img_back, 180, 200)
    print(np.shape(edges))
    edges = edges[border:height-border, :]
    print(np.shape(edges))

    cv2.imshow(".",edges)
    cv2.waitKey(0)

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
    cv2.imwrite("333.jpg", result)

    cv2.imshow("tst", result)
    cv2.waitKey(0)
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

image = cv2.imread("cam2/t.jpg")
depth, hm = itd_cvter.convert(image)
theta = hm2pos(hm)