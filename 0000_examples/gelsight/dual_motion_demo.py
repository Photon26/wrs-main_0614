import pickle
import robot_sim.robots.ur3_dual.ur3_dual as ur3d
import robot_con.ur.ur3_dual_x as ur3dx
import visualization.panda.world as wd
import modeling.geometric_model as gm
import numpy as np
import modeling.collision_model as cm
import cv2
import img_to_depth as itd
import time


inner_rad = 75/2
outer_rad = 99/2
center_rad = (inner_rad+outer_rad)/2

ur_dual_x = ur3dx.UR3DualX(lft_robot_ip='10.2.0.50', rgt_robot_ip='10.2.0.51', pc_ip='10.2.0.100')

base = wd.World(cam_pos=[2, 1, 3], lookat_pos=[0, 0, 1.1])
gm.gen_frame().attach_to(base)
robot_s = ur3d.UR3Dual()

pose_hnd = robot_s.get_gl_tcp(manipulator_name="lft_arm")
# print(pose_hnd)

ini_pos = np.array([0.3, 0,  1.4])
ini_rot_lft = np.array([[ 1, 0,  0],
       [ 0 , 0, -1],
       [ 0,  1,  0]])
# robot_meshmodel.attach_to(base)
center = ini_pos + np.dot(ini_rot_lft, np.array([0, -0.001*center_rad, 0]))

object = cm.CollisionModel("tape_2side_210618.stl")
object.set_pos(center)
object.set_rgba([.5, .7, .3, 1])
# object.attach_to(base)

ini_rot_rgt = np.array([[ 1, 0,  0],
       [ 0 , 0, 1],
       [ 0,  -1,  0]])

jnt_list = []
pre_jnt = robot_s.ik("rgt_arm", ini_pos, ini_rot_rgt, max_niter=1000)
for theta in range(3,9):
    rotmat = np.array([[np.cos(np.pi / 24 * theta + np.pi/4), 0, np.sin(np.pi / 24 * theta + np.pi/4)], [0, 1, 0],
                       [-np.sin(np.pi / 24 * theta + np.pi/4), 0, np.cos(np.pi / 24 * theta + np.pi/4)]])
    rot = np.dot(rotmat, ini_rot_rgt)
    pos = center + np.dot(rot, np.array([0, 0.001 * center_rad, 0]))
    newjnt_rgt = robot_s.ik("rgt_arm", pos, rot)
    if newjnt_rgt is None:
           newjnt_rgt = robot_s.ik("rgt_arm", pos, rot, pre_jnt, max_niter=1000)
    jnt_list.append(newjnt_rgt)
    pre_jnt = newjnt_rgt


# rgt hand hold the tape
# jnt_rgt = robot_s.ik("rgt_arm", ini_pos, ini_rot_rgt)
ini_jnt_rgt = jnt_list[0]
ur_dual_x.rgt_arm_hnd.move_jnts(ini_jnt_rgt)

#  loose lft hand a bit
# ini_pos_lft = ini_pos + np.dot(ini_rot_lft, np.array([0,0.005,0]))
ini_pos_lft = ini_pos + np.dot(ini_rot_lft, np.array([0, 0, -0.04]))
newjnt = robot_s.ik("lft_arm",ini_pos_lft, ini_rot_lft, max_niter=1000)
robot_s.fk("lft_arm", newjnt)
print(newjnt/3.14*180)
ur_dual_x.lft_arm_hnd.move_jnts(newjnt)
ur_dual_x.lft_arm_hnd.close_gripper(speedpercentange=20, forcepercentage=0) #   gripper control
# robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=False)
# robot_meshmodel.attach_to(base)

#  rotate rgt hand
count = 0
for jnt in jnt_list:
    if jnt is not None:
        print("*")
        ur_dual_x.rgt_arm_hnd.move_jnts(jnt)
        robot_s.fk("rgt_arm", jnt)
        if count <2:
            robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=False)
            robot_meshmodel.attach_to(base)
        count = count+1


base.run()
