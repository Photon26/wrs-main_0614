import pickle
import robot_sim.robots.ur3_dual.ur3_dual as ur3d
# import rbt_con.force_control as ur3dx
import robot_con.ur.ur3_dual_x as ur3dx
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
lft_jnt = np.array(ur_dual_x.get_jnt_values("lft_arm"))
print(lft_jnt)
# rgt_jnt = np.array([-1.01573354402651, -0.024298016224996388, 0.7465181350708008, -1.604677979146139, 1.9452768564224243, -3.5117350260363978])
# # lft_jnt_1 = np.array([1.0276005268096924, -2.85490066209902, -1.4274213949786585, 5.163119316101074, 4.332594871520996, 8.273784224187033])
#
# ur_dual_x.move_jnts("rgt_arm", rgt_jnt)
# # robot_s.fk("lft_arm", lft_jnt)
# pos, rot = robot_s.get_gl_tcp("rgt_arm")
# # pos[1] = pos[1]
# # pos[0] = pos[0] - 0.1
# new_jnt = robot_s.ik("rgt_arm", pos, rot, seed_jnt_values=rgt_jnt, max_niter=1000)
# print(new_jnt)
# robot_s.fk("rgt_arm", new_jnt)
# robot_meshmodel = robot_s.gen_meshmodel()
# robot_meshmodel.attach_to(base)
# base.run()
#
#
# lft_jnt_2 = np.array([0.8440605998039246, -3.249305073414938, -1.4211929480182093, 5.6194915771484375, 4.221709728240967, 8.435114685689108])
# ur_dual_x.move_jnts("lft_arm", lft_jnt_2)
# time.sleep(0.5)
# while ur_dual_x.lft_arm_hnd.arm.is_program_running():
#     pass
#
# lft_jnt_3 = np.array([0.39269161224365234, -2.1943357626544397, -2.079819981251852, 5.498745441436768, 4.005743980407715, 8.916003052388326])
# lft_jnt_4 = np.array([0.24308764934539795, -2.4585965315448206, -2.055237118397848, 5.86696720123291, 3.9646644592285156, 9.109419171010153])
# ur_dual_x.move_jnts("lft_arm", lft_jnt_4)
# rgt_jnt = ur_dual_x.get_jnt_values("rgt_arm")
# print(rgt_jnt)
# rgt_jnt_1 = np.array([-1.01573354402651, -0.024298016224996388, 0.7465181350708008, -1.604677979146139, 1.9452768564224243, -3.5117350260363978])
# # ur_dual_x.move_jnts("rgt_arm", rgt_jnt_1)
# robot_s.fk("rgt_arm", rgt_jnt_1)
# # robot_s.fk("rgt_arm", np.array(rgt_jnt))
# rgt_jnt_2 = np.array([-0.92263055, -0.60561254,  0.96072863,  0.13773314,  1.20871896, -2.66549251])

lft_jnt = np.array([0.3384285271167755, -2.4532974402057093, -1.0928729216205042, 4.816194534301758, 3.9879977703094482, 0])
ur_dual_x.move_jnts("lft_arm", lft_jnt)

# robot_s.ik

# # load model of tape
# object = cm.CollisionModel("tape.stl")
# object.set_pos(np.array([0.3, 0.1,  1.35]))
# object.set_scale([0.001, 0.001, 0.001])
# object.set_rgba([.5, .7, .3, 1])
# object.attach_to(base)
robot_s.fk("lft_arm", lft_jnt)
robot_meshmodel = robot_s.gen_meshmodel()
# robot_meshmodel.attach_to(base)
pos_1, rot = robot_s.get_gl_tcp("lft_arm")
print(pos_1)
print(rot, pos_1)
pos = pos_1[:].copy()
# pos[0] = pos_1
pos[1] = pos_1[1]+.1
# jnt = robot_s.ik("lft_arm", pos, rot,seed_jnt_values=lft_jnt, max_niter=10000)
# print(jnt)
# robot_s.fk("lft_arm", jnt)
robot_inik_solver = inik.IncrementalNIK(robot_s)
path = robot_inik_solver.gen_linear_motion("lft_arm",
                                               start_tcp_pos=pos_1,
                                               start_tcp_rotmat=rot,
                                               goal_tcp_pos=pos,
                                               goal_tcp_rotmat=rot,
                                           granularity=0.03,
                                               obstacle_list=[],
                                               seed_jnt_values=lft_jnt, )
# ur_dual_x.move_jnts("lft_arm", jnt)
# gm.gen_sphere(pos).attach_to(base)
print(path)
for jnt in path:
    robot_s.fk("lft_arm",jnt)
    robot_meshmodel = robot_s.gen_meshmodel()
    robot_meshmodel.attach_to(base)
# ur_dual_x.move_jspace_path("lft_arm", path)
# lft_pos, lft_rot = robot_s.get_gl_tcp("lft_arm")
#


base.run()