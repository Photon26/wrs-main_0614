import time
import robot_con.ur.ur3_rtq85_x as u3r85x
import robot_con.ur.program_builder as pb
import struct
import os
import numpy as np


class UR3DualX(object):
    """
    urx 50, right arm 51, left arm 52
    author: weiwei
    date: 20180131
    """

    def __init__(self, lft_robot_ip='10.2.0.50', rgt_robot_ip='10.2.0.51', pc_ip='10.2.0.100'):
        """
        :param robot_sim: for global transformation, especially in attachfirm
        author: weiwei
        date: 20191014 osaka
        """
        self._lft_arm_hnd = u3r85x.UR3Rtq85X(robot_ip=lft_robot_ip, pc_ip=pc_ip)
        self._rgt_arm_hnd = u3r85x.UR3Rtq85X(robot_ip=rgt_robot_ip, pc_ip=pc_ip)
        self._pb = pb.ProgramBuilder()
        self._script_dir = os.path.dirname(__file__)
        self._pb.load_prog(os.path.join(self._script_dir, "moderndriver_cbseries_master.script"))
        self._master_modern_driver_urscript = self._pb.get_program_to_run()
        self._master_modern_driver_urscript = self._master_modern_driver_urscript.replace("parameter_pc_ip",
                                                                                          self._lft_arm_hnd.pc_server_socket_info[
                                                                                              0])
        self._master_modern_driver_urscript = self._master_modern_driver_urscript.replace("parameter_pc_port",
                                                                                          str(
                                                                                              self._lft_arm_hnd.pc_server_socket_info[
                                                                                                  1]))
        self._master_modern_driver_urscript = self._master_modern_driver_urscript.replace("parameter_slave_ip",
                                                                                          rgt_robot_ip)
        self._master_modern_driver_urscript = self._master_modern_driver_urscript.replace("parameter_jnts_scaler",
                                                                                          str(self._lft_arm_hnd.jnts_scaler))
        self._pb.load_prog(os.path.join(self._script_dir, "moderndriver_cbseries_slave.script"))
        self._slave_modern_driver_urscript = self._pb.get_program_to_run()
        self._slave_modern_driver_urscript = self._slave_modern_driver_urscript.replace("parameter_master_ip",
                                                                                        lft_robot_ip)
        self._slave_modern_driver_urscript = self._slave_modern_driver_urscript.replace("parameter_jnts_scaler",
                                                                                        str(self._lft_arm_hnd.jnts_scaler))

        self._pb.load_prog(os.path.join(self._script_dir, "forcemode.script"))
        self._force_mode_driver_urscript = self._pb.get_program_to_run()
        self._force_mode_driver_urscript = self._force_mode_driver_urscript.replace("parameter_ip",
                                                                                    str(
                                                                                        self._lft_arm_hnd.pc_server_socket_info[
                                                                                            0]))
        self._force_mode_driver_urscript = self._force_mode_driver_urscript.replace("parameter_port",
                                                                                    str(
                                                                                        self._lft_arm_hnd.pc_server_socket_info[
                                                                                            1]))
        self._force_mode_driver_urscript = self._force_mode_driver_urscript.replace("parameter_jointscaler",
                                                                                    str(self._lft_arm_hnd.jnts_scaler))
        print(self._force_mode_driver_urscript)

    @property
    def lft_arm_hnd(self):
        # read-only property
        return self._lft_arm_hnd

    @property
    def rgt_arm_hnd(self):
        # read-only property
        return self._rgt_arm_hnd

    def move_jnts(self, component_name, jnt_values):
        """
        move all joints of the ur5 dual-arm robot_s
        NOTE that the two arms are moved sequentially
        :param component_name
        :param jnt_values: 1x6 or 1x12 array, depending on the value of component_name
        :return: bool

        author: weiwei
        date: 20170411
        """
        print('move_jnts')
        if component_name == "all":  # TODO Examine length, synchronization
            self._lft_arm_hnd.move_jnts(jnt_values[0:6], wait=False)
            self._rgt_arm_hnd.move_jnts(jnt_values[6:12], wait=True)
        elif component_name in ["lft_arm", "lft_hnd"]:
            self._lft_arm_hnd.move_jnts(jnt_values, wait=False)
        elif component_name in ["rgt_arm", "rgt_hnd"]:
            self._rgt_arm_hnd.move_jnts(jnt_values, wait=False)
        else:
            raise ValueError("Component_name must be in ['all', 'lft_arm', 'rgt_arm']!")

    def move_jspace_path(self,
                         component_name,
                         path,
                         control_frequency=.008,
                         interval_time=1.0,
                         interpolation_method=None):
        """
        :param component_name
        :param path: a list of 1x12 arrays or 1x6 arrays, depending on component_name
        :param control_frequency: the program will sample time_interval/control_frequency confs, see motion.trajectory
        :param interval_time: equals to expandis/speed, speed = degree/second
                              by default, the value is 1.0 and the speed is expandis/second
        :param interpolation_method
        :return:
        author: weiwei
        date: 20210404
        """
        if component_name == "all":
            self._lft_arm_hnd.trajt.set_interpolation_method(interpolation_method)
            interpolated_confs, interpolated_spds = self._lft_arm_hnd.trajt.piecewise_interpolation(path,
                                                                                                    control_frequency,
                                                                                                    interval_time)
            # upload a urscript to connect to the pc server started by this class
            self._rgt_arm_hnd.arm.send_program(self._slave_modern_driver_urscript)
            self._lft_arm_hnd.arm.send_program(self._master_modern_driver_urscript)
            # accept arm socket
            pc_server_socket, pc_server_socket_addr = self._lft_arm_hnd.pc_server_socket.accept()
            print("PC server connected by ", pc_server_socket_addr)
            # send trajectory
            keepalive = 1
            buf = bytes()
            for id, conf in enumerate(interpolated_confs):
                if id == len(interpolated_confs) - 1:
                    keepalive = 0
                jointsradint = [int(jnt_value * self._lft_arm_hnd.jnts_scaler) for jnt_value in conf]
                buf += struct.pack('!iiiiiiiiiiiii', jointsradint[0], jointsradint[1], jointsradint[2],
                                   jointsradint[3], jointsradint[4], jointsradint[5], jointsradint[6],
                                   jointsradint[7], jointsradint[8], jointsradint[9], jointsradint[10],
                                   jointsradint[11], keepalive)
            pc_server_socket.send(buf)
            pc_server_socket.close()
        elif component_name in ["lft_arm", "lft_hnd"]:
            self._lft_arm_hnd.move_jntspace_path(path=path,
                                                 control_frequency=control_frequency,
                                                 interval_time=interval_time,
                                                 interpolation_method=interpolation_method)
        elif component_name in ["rgt_arm", "rgt_hnd"]:
            self._rgt_arm_hnd.move_jntspace_path(path=path,
                                                 control_frequency=control_frequency,
                                                 interval_time=interval_time,
                                                 interpolation_method=interpolation_method)
        else:
            raise ValueError("Component_name must be in ['all', 'lft_arm', 'rgt_arm']!")

    def get_jnt_values(self, component_name):
        """
        get the joint angles of both arms
        :return: 1x12 array
        author: ochi, revised by weiwei
        date: 20180410, 20210404
        """
        if component_name == "all":
            return np.array(self._lft_arm_hnd.get_jnt_values() + self._rgt_arm_hnd.get_jnt_values())
        elif component_name in ["lft_arm", "lft_hnd"]:
            return self._lft_arm_hnd.get_jnt_values()
        elif component_name in ["rgt_arm", "rgt_hnd"]:
            return self._rgt_arm_hnd.get_jnt_values()
        else:
            raise ValueError("Component_name must be in ['all', 'lft_arm', 'rgt_arm']!")

    def force_mode_move_lft(self,
                            path,
                            control_frequency=.008,
                            interval_time=4.0,
                            interpolation_method=None):
        """
        :param component_name
        :param path: a list of 1x12 arrays or 1x6 arrays, depending on component_name
        :param control_frequency: the program will sample time_interval/control_frequency confs, see motion.trajectory
        :param interval_time: equals to expandis/speed, speed = degree/second
                              by default, the value is 1.0 and the speed is expandis/second
        :param interpolation_method
        :return:
        author: weiwei
        date: 20210404
        """
        interpolated_confs, interpolated_spds, _ = self._lft_arm_hnd.trajt.piecewise_interpolation(path,
                                                                                                   control_frequency,
                                                                                                   interval_time)
        print(len(interpolated_confs))
        # upload a urscript to connect to the pc server started by this class
        self._lft_arm_hnd.arm.send_program(self._force_mode_driver_urscript)
        # self._lft_arm_hnd.arm.send_program(self._master_modern_driver_urscript)
        # accept arm socket
        pc_server_socket, pc_server_socket_addr = self._lft_arm_hnd._pc_server_socket.accept()
        print("PC server onnected by ", pc_server_socket_addr)
        # send trajectory
        keepalive = 1
        buf = bytes()
        for id, conf in enumerate(interpolated_confs):
            if id == len(interpolated_confs) - 1:
                keepalive = 0
            jointsradint = [int(jnt_value * self._lft_arm_hnd.jnts_scaler) for jnt_value in conf]
            buf += struct.pack('!iiiiiii', jointsradint[0], jointsradint[1], jointsradint[2],
                               jointsradint[3], jointsradint[4], jointsradint[5], keepalive)
        pc_server_socket.send(buf)
        pc_server_socket.close()


if __name__ == '__main__':
    import motion.optimization_based.incremental_nik as inik
    import visualization.panda.world as wd
    import modeling.geometric_model as gm
    import modeling.collision_model as cm
    import robot_sim.robots.ur3_dual.ur3_dual as ur3d
    import numpy as np
    import math
    import basis.robot_math as rm

    base = wd.World(cam_pos=[2, 1, 3], lookat_pos=[0, 0, 1.1])
    gm.gen_frame().attach_to(base)
    # robot_s
    component_name = 'lft_arm'
    robot_instance = ur3d.UR3Dual()

    u3r85dx = UR3DualX(lft_robot_ip='10.2.0.50', rgt_robot_ip='10.2.0.51', pc_ip='10.2.0.100')
    start_jnts = np.asarray(u3r85dx.get_jnt_values(component_name='lft_arm'))
    robot_instance.fk(component_name, start_jnts)
    tcp_pos, tcp_rot = robot_instance.get_gl_tcp(component_name)
    goal_hnd_pos = tcp_pos + np.array([0, .05, 0])
    goal_hnd_rotmat = tcp_rot

    robot_inik_solver = inik.IncrementalNIK(robot_instance)
    path = robot_inik_solver.gen_linear_motion(component_name,
                                               start_tcp_pos=tcp_pos,
                                               start_tcp_rotmat=tcp_rot,
                                               goal_tcp_pos=goal_hnd_pos,
                                               goal_tcp_rotmat=goal_hnd_rotmat,
                                               obstacle_list=[],
                                               seed_jnt_values=start_jnts)

    for jnt_values in path:
        print(jnt_values)
        robot_instance.fk(component_name, jnt_values)
        robot_meshmodel = robot_instance.gen_meshmodel()
        robot_meshmodel.attach_to(base)
    # base.run()

    # u3r85dx.move_jnts('lft_arm', path[0])
    # u3r85dx.move_jnts('lft_arm', path[1])
    # u3r85dx.move_jspace_path('lft_arm', path=path)
    u3r85dx.force_mode_move_lft(path=path)
