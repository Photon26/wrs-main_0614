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
print(ur_dual_x.get_jnt_values("rgt_arm"))

base = wd.World(cam_pos=[2,1,3], lookat_pos=[0,0,1.1])
gm.gen_frame().attach_to(base)
robot_s = ur3d.UR3Dual()
robot_inik_solver = inik.IncrementalNIK(robot_s)

lft_jnt = ur_dual_x.get_jnt_values("lft_arm")
print(lft_jnt)
print("iii")
rgt_jnt = ur_dual_x.get_jnt_values("rgt_arm")

lft_jnt = [0.5917282104492188, -2.404630486165182, -0.7898209730731409, 2.3452978134155273, 4.231568813323975, 7.110745493565695]
rgt_jnt = [-0.7641804854022425, -0.7214272657977503, 0.7556247711181641, 0.5889056921005249, 1.7192022800445557, 3.644009590148926]
robot_s.fk("lft_arm", np.array(lft_jnt))
robot_s.fk("rgt_arm", np.array(rgt_jnt))
rgt_pos, rgt_rot = robot_s.get_gl_tcp("rgt_hnd")
lft_pos, lft_rot = robot_s.get_gl_tcp("lft_hnd")
tapeend = rgt_pos + np.dot(rgt_rot, np.array([0.094, -1*tape_rad, 0]))
lft_pos = tapeend + np.dot(lft_rot, np.array([0,0,-0.055]))
# lft_stpos = lft_pos + np.dot(lft_rot, np.array([0,0,-0.02]))
# lft_endpos = lft_pos + np.dot(lft_rot, np.array([0,0,0.02]))
pos_waypoint = lft_pos + np.dot(lft_rot, np.array([0,-0.02,0.01]))
lft_wayjnt = robot_s.ik("lft_arm", pos_waypoint, lft_rot, max_niter=1000, seed_jnt_values=np.array(lft_jnt))
print(lft_wayjnt)
# lft_endjnt = robot_s.ik("lft_arm", lft_endpos, lft_rot, max_niter= 1000)
# pose_list = robot_inik_solver.gen_linear_motion("lft_arm",
#                                                     start_tcp_pos=lft_stpos,
#                                                     start_tcp_rotmat=lft_rot,
#                                                     goal_tcp_pos=lft_endpos,
#                                                     goal_tcp_rotmat=lft_rot,
#                                                     obstacle_list=[])
# path_1 = robot_inik_solver.gen_linear_motion("lft_arm",
#                                                start_tcp_pos=lft_stpos,
#                                                start_tcp_rotmat=lft_rot,
#                                                goal_tcp_pos=lft_endpos,
#                                                goal_tcp_rotmat=lft_rot,
#                                                obstacle_list=[],
#                                                seed_jnt_values=np.array(lft_jnt))
#
# path_2 = robot_inik_solver.gen_linear_motion("lft_arm",
#                                                start_tcp_pos=lft_endpos,
#                                                start_tcp_rotmat=lft_rot,
#                                                goal_tcp_pos=lft_stpos,
#                                                goal_tcp_rotmat=lft_rot,
#                                                obstacle_list=[],
#                                                seed_jnt_values=np.array(lft_endjnt))

ur_dual_x.move_jnts("lft_arm", lft_jnt)
# ur_dual_x.move_jnts("rgt_arm", rgt_jnt)
lft_jnt_end = np.array([0.6170846223831177, -2.4741104284869593, -0.7091301123248499, 2.3565778732299805, 4.262228012084961, 7.0119651595698755])
lft_wayjnt = np.array([0.6037420630455017, -2.4415715376483362, -0.7639496962176722, 2.3737130165100098, 4.253231525421143, 7.001009706650869])
# print(path_1)

rrtc_planner = rrtc.RRTConnect(robot_s)
path = rrtc_planner.plan(component_name="lft_arm",
                         start_conf=np.array(np.array(lft_jnt)),
                         goal_conf=lft_jnt_end,
                         ext_dist=.05,
                         max_time=1000)

# print(path)
for jnt_values in path:
    robot_s.fk("lft_arm", jnt_values)
    robot_meshmodel = robot_s.gen_meshmodel()
    robot_meshmodel.attach_to(base)

robot_s.fk("lft_arm", lft_wayjnt)
robot_meshmodel = robot_s.gen_meshmodel()
robot_meshmodel.attach_to(base)

# time.sleep(0.5)
# while ur_dual_x.lft_arm_hnd.arm.is_program_running():
#     pass
# ur_dual_x.force_mode_move_lft(path= path, control_frequency=.1)
#
# time.sleep(0.5)
# while ur_dual_x.lft_arm_hnd.arm.is_program_running():
#     pass
# ur_dual_x.move_jnts("lft_arm", lft_wayjnt)
#
# time.sleep(0.5)
# while ur_dual_x.lft_arm_hnd.arm.is_program_running():
#     pass
# ur_dual_x.move_jnts("lft_arm", lft_jnt)


#     ur_dual_x.force_mode_move_lft(path=path_2)

# gm.gen_sphere(tapeend).attach_to(base)

# print(lft_jnt)
# [0.5558556914329529, -2.3695347944842737, -0.8094676176654261, 2.3152518272399902, 4.208155632019043, 7.079980496560232]
# print(rgt_jnt)
# [-0.7934578100787562, -0.6980989615069788, 0.8795714378356934, 0.4155263900756836, 1.7236570119857788, 3.7133867740631104]

# robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=True)
# robot_meshmodel.attach_to(base)



base.run()