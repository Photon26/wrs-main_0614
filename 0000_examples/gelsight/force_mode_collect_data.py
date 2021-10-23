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

tape_rad = (0.094 + 0.076)/4
ur_dual_x = ur3dx.UR3DualX(lft_robot_ip='10.2.0.50', rgt_robot_ip='10.2.0.51', pc_ip='10.2.0.100')

base = wd.World(cam_pos=[2,1,3], lookat_pos=[0,0,1.1])
gm.gen_frame().attach_to(base)
robot_s = ur3d.UR3Dual()

lft_jnt = ur_dual_x.get_jnt_values("lft_arm")
rgt_jnt = ur_dual_x.get_jnt_values("rgt_arm")
robot_s.fk("lft_arm", np.array(lft_jnt))
robot_s.fk("rgt_arm", np.array(rgt_jnt))
lft_pos, lft_rot = robot_s.get_gl_tcp("rgt_hnd")
tapeend = lft_pos + np.dot(lft_rot, np.array([0.094, -1*tape_rad, 0]))
gm.gen_sphere(tapeend).attach_to(base)

print(lft_jnt)
# [0.5558556914329529, -2.3695347944842737, -0.8094676176654261, 2.3152518272399902, 4.208155632019043, 7.079980496560232]
print(rgt_jnt)
# [-0.7934578100787562, -0.6980989615069788, 0.8795714378356934, 0.4155263900756836, 1.7236570119857788, 3.7133867740631104]

robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=True)
robot_meshmodel.attach_to(base)

base.run()