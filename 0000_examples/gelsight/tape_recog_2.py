"""
    for single hnd searching for end(21.12.7)
"""

import pickle
import robot_sim.robots.ur3_dual.ur3_dual as ur3d
import rbt_con.force_control as ur3dx
# import robot_con.ur.ur3_dual_x as ur3dx
import visualization.panda.world as wd
import modeling.geometric_model as gm
import motion.optimization_based.incremental_nik as inik
import numpy as np
import modeling.collision_model as cm
import cv2
import img_to_depth as itd
import time
import motion.probabilistic.rrt_connect as rrtc

tape_rad = (0.094 + 0.076)/4
ur_dual_x = ur3dx.UR3DualX(lft_robot_ip='10.2.0.50', rgt_robot_ip='10.2.0.51', pc_ip='10.2.0.100')

base = wd.World(cam_pos=[2,1,3], lookat_pos=[0,0,1.1])
gm.gen_frame().attach_to(base)
robot_s = ur3d.UR3Dual()
robot_inik_solver = inik.IncrementalNIK(robot_s)

# lft_jnt = ur_dual_x.get_jnt_values("lft_arm")
# print(lft_jnt)
lft_jnt = np.array([0.3384285271167755, -2.4532974402057093, -1.0928729216205042, 4.816194534301758, 3.9879977703094482, 8.985321823750631])
ur_dual_x.move_jnts("lft_arm", lft_jnt)
robot_s.fk("lft_arm", lft_jnt)
robot_meshmodel = robot_s.gen_meshmodel()
robot_meshmodel.attach_to(base)
lft_pos, lft_rot = robot_s.get_gl_tcp("lft_arm")
new_jnt = lft_jnt.copy()
new_jnt[5] = new_jnt[5]+30/180*np.pi
while(True):
    time.sleep(0.5)
    while ur_dual_x.lft_arm_hnd.arm.is_program_running():
        pass
    ur_dual_x.lft_arm_hnd.close_gripper(speedpercentange=20, forcepercentage=80)  # gripper control

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

robot_s.fk("lft_arm", new_jnt)
robot_meshmodel = robot_s.gen_meshmodel()
robot_meshmodel.attach_to(base)

base.run()