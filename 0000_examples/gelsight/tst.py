import pickle
import robot_sim.robots.ur3_dual.ur3_dual as ur3d
import robot_con.ur.ur3_dual_x as ur3dx
import visualization.panda.world as wd
import modeling.geometric_model as gm
import numpy as np
import cv2
import img_to_depth as itd

inner_rad = 75/2
outer_rad = 99/2

robot_instance = ur3d.UR3Dual()
ur_dual_x = ur3dx.UR3DualX(lft_robot_ip='10.2.0.50', rgt_robot_ip='10.2.0.51', pc_ip='10.2.0.100')



base = wd.World(cam_pos=[2, 1, 3], lookat_pos=[0, 0, 1.1])
gm.gen_frame().attach_to(base)
component_name = "lft_arm"

jnt = ur_dual_x.lft_arm_hnd.get_jnt_values()
robot_instance.fk(component_name,np.array(jnt))
pose_hnd = robot_instance.get_gl_tcp(manipulator_name=component_name)
print(pose_hnd)
print(jnt)
print("shi fou peng zhuang",robot_instance.is_collided())
robot_meshmodel = robot_instance.gen_meshmodel(toggle_tcpcs=True)
robot_meshmodel.attach_to(base)

pos = np.array([ 0.3, 0,  1.4])

ini_rot_rgt = np.array([[ 1, 0,  0],
       [ 0 , 0, 1],
       [ 0,  -1,  0]])
ini_rot_lft = np.array([[ 1, 0,  0],
       [ 0 , 0, -1],
       [ 0,  1,  0]])
gm.gen_frame(pos=pose_hnd[0],rotmat=pose_hnd[1],length=0.4).attach_to(base)
print(robot_instance.ik("lft_arm",tgt_pos=pose_hnd[0],tgt_rotmat=pose_hnd[1],seed_jnt_values=jnt,max_niter=1000))
base.run()


# ini_rot_lft = np.array([[ 1, 0,  0],
#        [ 0 , 0, -1],
#        [ 0,  1,  0]])
# ini_rot_rgt = np.array([[ 1, 0,  0],
#        [ 0 , 0, 1],
#        [ 0,  -1,  0]])
# i = 0
# # poslist = []
# # while (i < 1):
# #     j = 1.1
# #     while j < 1.6:
# #         pos = np.array([i, 0, j])
# #
# #         jnt = robot_instance.lft_arm.ik(pos, ini_rot_lft)
# #         jnt2 = robot_instance.rgt_arm.ik(pos, ini_rot_rgt)
# #         # if jnt is not None and jnt2 is not None:
# #         if jnt2 is not None:
# #             print(i,j)
# #             poslist.append([i,j])
# #         j = j+0.05
# #     i = i+0.05
# #
# # print(poslist)
#
# base = wd.World(cam_pos=[2, 1, 3], lookat_pos=[0, 0, 1.1])
# gm.gen_frame().attach_to(base)
#
# # jnt = ur_dual_x.lft_arm_hnd.get_jnt_values()
# # robot_instance.fk("lft_arm",np.array(jnt))
# # pose_hnd = robot_instance.get_gl_tcp(manipulator_name="lft_arm")
# # print(pose_hnd)
# # print(jnt)
# # print("shi fou peng zhuang",robot_instance.is_collided())
# pos = np.array([0,0,1.15])
# jnt = robot_instance.rgt_arm.ik(pos, ini_rot_rgt)
# robot_instance.fk("rgt_arm", jnt)
# robot_meshmodel = robot_instance.gen_meshmodel(toggle_tcpcs=True)
# robot_meshmodel.attach_to(base)
# print(robot_instance.is_collided())
# # ur_dual_x.move_jnts("rgt_arm", jnt)
# # print(robot_instance.ik("lft_arm",tgt_pos=pose_hnd[0],tgt_rotmat=pose_hnd[1]))
# base.run()