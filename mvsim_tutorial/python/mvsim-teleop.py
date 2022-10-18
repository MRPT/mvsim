#!/usr/bin/env python3

# ---------------------------------------------------------------------
# This example shows how to:
# - Call a MVSIM service.
# - Use it to drive a vehicle/robot using its internal kinematics
#   controller.
#
# Install python3-mvsim, or test with a local build with:
# export PYTHONPATH=$HOME/code/mvsim/build/:$PYTHONPATH
# ---------------------------------------------------------------------

from mvsim_comms import pymvsim_comms
from mvsim_msgs import SrvSetControllerTwist_pb2
import time
import math


def sendRobotTwistSetpoint(client, robotName, vx, vy, w):
    # (vx,vy) in local coordinates [m/s]
    # (w) in [rad/s]

    req = SrvSetControllerTwist_pb2.SrvSetControllerTwist()
    req.objectId = robotName  # vehicle/robot/object name in MVSIM
    req.twistSetPoint.vx = vx
    req.twistSetPoint.vy = vy
    req.twistSetPoint.vz = 0
    req.twistSetPoint.wx = 0
    req.twistSetPoint.wy = 0
    req.twistSetPoint.wz = w
    # ret =
    client.callService('set_controller_twist', req.SerializeToString())


if __name__ == "__main__":
    client = pymvsim_comms.mvsim.Client()
    client.setName("mvsim-teleop")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")

    sendRobotTwistSetpoint(client, 'r1', 0.75, 0.0, 1.0)
