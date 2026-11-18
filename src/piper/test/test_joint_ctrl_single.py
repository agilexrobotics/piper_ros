# Copyright (c) 2026 Jayden Vasquez
#
# SPDX-License-Identifier: MIT

"""joint_ctrl_single: what reaches the SDK for a given JointState command."""

from piper.piper_ctrl_single_node import PiperRosNode
import pytest
from sensor_msgs.msg import JointState

FACTOR = 57324.840764  # rad -> 0.001 deg, as in joint_callback
POSE = [0.5, 1.0, -1.0, 0.3, 0.5, 0.2]


class FakePiper:

    def __init__(self):
        self.joint_cmds = []
        self.gripper_cmds = []

    def MotionCtrl_2(self, *args):
        pass

    def JointCtrl(self, *joints):
        self.joint_cmds.append(joints)

    def GripperCtrl(self, *args):
        self.gripper_cmds.append(args)


class FakeLogger:

    def __init__(self):
        self.errors = []

    def info(self, *args, **kwargs):
        pass

    def error(self, msg, **kwargs):
        self.errors.append(msg)


def make_node():
    """Node with the SDK replaced, without opening a CAN port or a ROS context."""
    node = PiperRosNode.__new__(PiperRosNode)
    node.piper = FakePiper()
    node.gripper_exist = True
    node.gripper_val_mutiple = 1
    logger = FakeLogger()
    node.get_logger = lambda: logger
    node.GetEnableFlag = lambda: True
    return node, logger


def command(names, positions):
    msg = JointState()
    msg.name = names
    msg.position = positions
    return msg


def test_full_command_is_sent():
    node, logger = make_node()
    names = [f'joint{i}' for i in range(1, 8)]
    node.joint_callback(command(names, POSE + [0.03]))
    assert node.piper.joint_cmds == [tuple(round(p * FACTOR) for p in POSE)]
    assert node.piper.gripper_cmds[0][0] == 30000
    assert logger.errors == []


@pytest.mark.parametrize('names,positions', [
    # names with a prefix (namespaced / multi-arm description)
    ([f'piper_joint{i}' for i in range(1, 8)], POSE + [0.03]),
    # partial command: only joint1
    (['joint1'], [0.5]),
    # one arm joint left out
    (['joint1', 'joint2', 'joint3', 'joint4', 'joint6'], [0.5, 1.0, -1.0, 0.3, 0.2]),
    # no names at all
    ([], POSE + [0.03]),
])
def test_command_missing_an_arm_joint_is_not_sent(names, positions):
    node, logger = make_node()
    node.joint_callback(command(names, positions))
    # before the fix, the missing joints went out as 0 rad
    assert node.piper.joint_cmds == []
    assert node.piper.gripper_cmds == []
    assert len(logger.errors) == 1
