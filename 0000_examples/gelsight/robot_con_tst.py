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

ur_dual_x = ur3dx.UR3DualX(lft_robot_ip='10.2.0.50', rgt_robot_ip='10.2.0.51', pc_ip='10.2.0.100')

base = wd.World(cam_pos=[2,1,3], lookat_pos=[0,0,1.1])
gm.gen_frame().attach_to(base)
robot_s = ur3d.UR3Dual()
# lft_jnt = ur_dual_x.get_jnt_values("lft_arm")
# print(lft_jnt)
lft_jnt = np.array([0.3384285271167755, -2.4532974402057093, -1.0928729216205042, 4.816194534301758, 3.9879977703094482, 8.985321823750631])
ur_dual_x.move_jnts("lft_arm", lft_jnt)
robot_s.fk("lft_arm", lft_jnt)
robot_meshmodel = robot_s.gen_meshmodel()
robot_meshmodel.attach_to(base)
lft_pos, lft_rot = robot_s.get_gl_tcp("lft_arm")

robot_meshmodel = robot_s.gen_meshmodel()
robot_meshmodel.attach_to(base)

base.run()