#!/usr/bin/env python

from struct import *
import time
import socket
import struct


from ..manipulator import Manipulator
from ..utils import *


class Ur3(Manipulator):

    def __init__(self, ip, port_read, port_write):
        """
        Connects the UR3 robot.
        :param ip: IP of a robot.
        :param port_read: Port for reading data from robot.
        :param port_write: Port for writing data to robot.
        """
        Manipulator.__init__(self, ip, port_read, port_write)
        self.chunk_size = 8
        self.num_chunks = 6
        self.start_chunk_cartesian = 444
        self.start_chunk_joint = 252
        self.stop_chunk_cartesian = self.start_chunk_cartesian + self.num_chunks * self.chunk_size
        self.stop_chunk_joint = self.start_chunk_joint + self.num_chunks * self.chunk_size
        self.ur_package_size = 1060
        self.vec = None

#get_pose based on http://www.zacobria.com/universal-robots-knowledge-base-tech-support-forum-hints-tips/knowledge-base/client-interfaces-cartesian-matlab-data/?fbclid=IwAR1ZMKLu1ioCA3yiwE80Tzgbye-LZIq1gVxqvsJw0B9-Pbm2IE7Hitev41w 
# by Zacobria Lars Skovsgaard
    @robot_command
    @robot_command
    def get_pose(self):
        """
        Returns a pose of a connected robot.
        :return: np.array with 6 elements: [x, y, z, a, b, c]
        """

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self.ip, self.port_read))
        time.sleep(1.00)
        packet_1 = s.recv(4)
        packet_2 = s.recv(8)
        packet_3 = s.recv(48)
        packet_4 = s.recv(48)
        packet_5 = s.recv(48)
        packet_6 = s.recv(48)
        packet_7 = s.recv(48)
        packet_8 = s.recv(48)
        packet_9 = s.recv(48)
        packet_10 = s.recv(48)
        packet_11 = s.recv(48)
        packet_12 = s.recv(8)
        packet_12 = packet_12.encode("hex")  # convert the data from \x hex notation to plain hex
        x = str(packet_12)
        x = struct.unpack('!d', packet_12.decode('hex'))[0]

        packet_13 = s.recv(8)
        packet_13 = packet_13.encode("hex")  # convert the data from \x hex notation to plain hex
        y = str(packet_13)

        y = struct.unpack('!d', packet_13.decode('hex'))[0]

        packet_14 = s.recv(8)
        packet_14 = packet_14.encode("hex")  # convert the data from \x hex notation to plain hex
        z = str(packet_14)
        z = struct.unpack('!d', packet_14.decode('hex'))[0]

        packet_15 = s.recv(8)
        packet_15 = packet_15.encode("hex")  # convert the data from \x hex notation to plain hex
        Rx = str(packet_15)
        Rx = struct.unpack('!d', packet_15.decode('hex'))[0]

        packet_16 = s.recv(8)
        packet_16 = packet_16.encode("hex")  # convert the data from \x hex notation to plain hex
        Ry = str(packet_16)
        Ry = struct.unpack('!d', packet_16.decode('hex'))[0]

        packet_17 = s.recv(8)
        packet_17 = packet_17.encode("hex")  # convert the data from \x hex notation to plain hex
        Rz = str(packet_17)
        Rz = struct.unpack('!d', packet_17.decode('hex'))[0]

        pose = [x, y, z, Rx, Ry, Rz]

        return np.asarray(pose)

    @robot_command
    def get_joints(self, deg=False):
        """
        Returns joint coordinates of a connected robot.
        :return: np.array with 6 elements [j1, j2, j3, j4, j5, j6]
        """
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self.ip, self.port_read))
        time.sleep(1.00)
        packet_1 = s.recv(4)
        packet_2 = s.recv(8)
        packet_3 = s.recv(48)
        packet_4 = s.recv(48)
        packet_5 = s.recv(48)
        packet_6 = s.recv(48)
        packet_7 = s.recv(48)
        packet_8 = s.recv(8)
        packet_8 = packet_8.encode("hex")  # convert the data from \x hex notation to plain hex
        q1 = str(packet_8)
        q1 = struct.unpack('!d', packet_8.decode('hex'))[0]

        packet_9 = s.recv(8)
        packet_9 = packet_9.encode("hex")  # convert the data from \x hex notation to plain hex
        q2 = str(packet_9)

        q2 = struct.unpack('!d', packet_9.decode('hex'))[0]

        packet_10 = s.recv(8)
        packet_10 = packet_10.encode("hex")  # convert the data from \x hex notation to plain hex
        q3 = str(packet_10)
        q3 = struct.unpack('!d', packet_10.decode('hex'))[0]

        packet_11 = s.recv(8)
        packet_11 = packet_11.encode("hex")  # convert the data from \x hex notation to plain hex
        q4 = str(packet_11)
        q4 = struct.unpack('!d', packet_11.decode('hex'))[0]

        packet_12 = s.recv(8)
        packet_12 = packet_12.encode("hex")  # convert the data from \x hex notation to plain hex
        q5 = str(packet_12)
        q5 = struct.unpack('!d', packet_12.decode('hex'))[0]

        packet_13 = s.recv(8)
        packet_13 = packet_13.encode("hex")  # convert the data from \x hex notation to plain hex
        q6 = str(packet_13)
        q6 = struct.unpack('!d', packet_13.decode('hex'))[0]

        pose = [q1, q2, q3, q4, q5, q6]

    # @robot_command
    def move(self, trajectory, is_movej=True, is_pose=True, a=1, v=1, use_mapping=False):
        """
        Moves a connected robot.
        :param trajectory: list of poses/joint coordinates.
        :param is_movej: bool
        :param is_pose: bool
        :param a: acceleration
        :param v: velocity
        :param use_mapping: bool. Specify if you want to set points in the external coordinate system (see set_mapping(..))
        :return: executed commands in an order (text)
        """
        assert isinstance(trajectory, list)
        assert len(trajectory) > 0
        assert isinstance(is_movej, bool)
        assert isinstance(is_pose, bool)
        assert isinstance(trajectory[0], np.ndarray)
        assert trajectory[0].size == 6

        # if user provides coordinates expressed not in robot system transform them to proper system
        if use_mapping:
            assert self.coordinates_mapping is not None
            assert self.coordinates_mapping.shape == (4, 4)

        print "Trajectory has {0} target points".format(len(trajectory))

        commands = list()
        for i, point in enumerate(trajectory):
            command = self.create_move_command(point, is_movej, is_pose, a, v, use_mapping)

            self.socket_write.send(command)
            print command

            self.wait_for_move_end(point, is_pose)
            print "Achieved {0} target points".format(i)
            commands.append(command)
        return commands

    def create_move_command(self, point, is_movej=True, is_pose=True, a=1, v=1, use_mapping=False):
        """
        Helper that creates the command for movement without executing it. Can be used for force mode.
        :param point: point in space expressed as a pose XYZABC or joints J1:6
        :param is_movej: bool
        :param is_pose: bool
        :param a: acceleration
        :param v: velocity
        :param use_mapping: bool. Specify if you want to set points in the external coordinate system (see set_mapping(..))
        :return: created command (text)
        """
        assert isinstance(point, np.ndarray)
        assert point.size == 6
        assert isinstance(is_movej, bool)
        assert isinstance(is_pose, bool)

        command = ""
        if is_movej:
            command += "movej("
        else:
            command += "movel("

        if is_pose:
            command += "p"

        if use_mapping and is_pose:
            point = mat2pose(np.linalg.inv(self.coordinates_mapping).dot(pose2mat(point)))

        print point
        command += "[{}, {}, {}, {}, {}, {}], ".format(*point)
        command += "a={0},v={1}".format(a, v)
        command += ")\n"

        return command

    def wait_for_move_end(self, target_position, is_pose):
        start_time = time.time()
        while True:
            is_in_position = True
            if is_pose:
                current_position = self.get_pose()
            else:
                current_position = self.get_joints()

            for i, el in enumerate(target_position):
                if (target_position[i] < current_position[i] - 0.07) or (
                        target_position[i] > current_position[i] + 0.07):
                    is_in_position = False
            elapsed_time = time.time() - start_time
            if is_in_position or elapsed_time > 5:
                break

    @robot_command
    def grip(self, range_open):
        """
        Sends command to open the RG6 gripper.
        :param range_open: number in [cm]
        :return: void
        """
        assert isinstance(range_open, float) or isinstance(range_open, int)
        command = rg6_cmd(range_open)
        self.socket_write.send(command)

    @robot_command
    def write_custom_command(self, command):
        self.socket_write.send(command)

    def get_data_from_ur3_package(self, msg, start_byte, stop_byte, num_chunks, chunk_size):
        values_bytes = msg[start_byte:stop_byte]
        values = list()
        for i in range(num_chunks):
            m = chunk_size * i

            # byte string containing 8-byte double number
            value = b''
            for k in range(chunk_size, 0, -1):
                value += values_bytes[(k - 1) + m]

            values.append(unpack('d', value)[0])
        return values

    @robot_command
    def execute_in_force_mode(self, trajectory_commands,
                              task_frame=np.asarray([0, 0, 0, 0, 0, 0]), selection_vector=np.asarray([0, 0, 1, 0, 0, 0]),
                              wrench=np.asarray([0.0, 0.0, 5.0, 0.0, 0.0, 0.0]), type=1,
                              limits=np.asarray([0.1, 0.1, 0.15, 0.3490658503988659, 0.3490658503988659, 0.3490658503988659])):
        """
        Executes constructed commands in the specified force mode. Please see URScript API reference doc - force_mode(). All
        values must be provided as numpy arrays.
        :param trajectory_commands: list of commands that will be executed using force mode in string
        :param task_frame: A pose vector that defines the force frame relative to the base frame.
        :param selection_vector: A 6d vector of 0s and 1s. 1 means that the robot will be compliant in the corresponding axis of the task frame.
        :param wrench: The forces/torques the robot will apply to its environment.
        :param type: An integer [1;3] specifying how the robot interprets the force frame.
        :param limits:  6d vector. For compliant axes, these values are the maximum allowed tcp speed along/about the axis.
        :return: executed command
        """

        assert isinstance(trajectory_commands, list) and len(trajectory_commands) > 0 and \
               isinstance(trajectory_commands[0], str)
        assert isinstance(task_frame, np.ndarray) and isinstance(selection_vector, np.ndarray) and \
               isinstance(wrench, np.ndarray) and isinstance(limits, np.ndarray)

        force_mode_path = "def myProg():\n" \
                          "\tforce_mode(p{0}, {1}, {2}, {3}, {4})\n" \
                          "\tsleep(0.5)\n".format(np.array2string(task_frame, separator=','),
                                                  np.array2string(selection_vector, separator=','),
                                                  np.array2string(wrench, separator=','),
                                                  type,
                                                  np.array2string(limits, separator=','))

        for cmd in trajectory_commands:
            force_mode_path += "\t" + cmd + "\tsleep(2.0)\n"
        force_mode_path += "end\n"

        self.write_custom_command(force_mode_path)

        time.sleep(5)
        self.write_custom_command("end_force_mode()\n")

        return force_mode_path + "end_force_mode()\n"

