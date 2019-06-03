#!/usr/bin/env python

"""
Interface for manipulators that can be connected via TCP/IP sockets.
"""

import socket

import numpy as np


class Manipulator:
    def __init__(self, ip, port_read, port_write):
        """ Opens connection between a root and a host robot """
        assert isinstance(ip, str)
        assert isinstance(port_read, int) or isinstance(port_read, str)
        assert isinstance(port_write, int) or isinstance(port_write, str)
        if type(port_read) is str:
            port_read = int(port_read)

        if type(port_write) is str:
            port_write = int(port_write)

        self.ip = ip
        self.port_write = port_write
        self.port_read = port_read
        self.coordinates_mapping = None

        try:
            self.socket_read = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket_write = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket_read.connect((self.ip, self.port_read))
            self.socket_write.connect((self.ip, self.port_write))
            self.socket_write.settimeout(5)
            self.socket_read.settimeout(5)
            print("[Socket -- IP: {0}. Write port: {1}, read port: {2}]\n".format(ip, self.port_write, self.port_read))
        except socket.error, exc:
            print("[Socket cannot be created. Exception occured:\n{0}]\n".format(exc))

    def __del__(self):
        """ Closes sockets created during initialization. """
        self.socket_read.close()
        self.socket_write.close()
        print("\n[Sockets succesfully closed.]\n".format(self.ip))

    def set_mapping(self, matrix):
        """
        Set mapping between robot and some external coordinate systems.
        :param matrix: Homogeneous matrix np.array() 4x4
        :return: void
        """
        assert isinstance(matrix, np.ndarray)
        assert matrix.shape == (4, 4)
        self.coordinates_mapping = matrix
        print("Coordinates mapping will be applied:\n{0}\n From now on specify robot's pose in your coordinate system.".format(matrix))

    def reset_mapping(self):
        """ Removes homogeneous matrix of transformation between a robot and external camera coordinate system. """
        self.coordinates_mapping = None

    def get_mapping(self):
        """ Sets homogeneous matrix of transformation between a robot and external camera coordinate system. """
        return self.coordinates_mapping

    def grip(self, range_open):
        pass

    def move(self, *args):
        raise NotImplementedError("Implement this method")

    def get_pose(self):
        raise NotImplementedError("Implement this method")

    def get_joints(self):
        raise NotImplementedError("Implement this method")

    def execute_in_force_mode(self, *args):
        raise NotImplementedError("Implement this method")
