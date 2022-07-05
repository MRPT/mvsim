#!/usr/bin/env python3

# ---------------------------------------------------------------------
# This example shows how to:
# - Call a MVSIM service.
# - Use it to move an object in an arbitrary way.
#
# Install python3-mvsim, or test with a local build with:
# export PYTHONPATH=$HOME/code/mvsim/build/lib/:$PYTHONPATH
# ---------------------------------------------------------------------

from mvsim_comms import pymvsim_comms
from mvsim_msgs import SrvSetPose_pb2
import time
import math


def setObjectPose(client, objectName, x, y, theta_rad):
    # Send a set pose request:
    req = SrvSetPose_pb2.SrvSetPose()
    req.objectId = objectName  # vehicle/robot/object name in MVSIM
    req.pose.x = x
    req.pose.y = y
    req.pose.z = 0
    req.pose.yaw = theta_rad
    req.pose.pitch = 0
    req.pose.roll = 0  # math.radians(0.0)
    # ret =
    client.callService('set_pose', req.SerializeToString())


if __name__ == "__main__":
    client = pymvsim_comms.mvsim.Client()
    client.setName("tutorial1")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")

    for i in range(1000):
        th = i*0.02
        R = 5
        setObjectPose(client, 'r1', math.sin(th)*R, R*(1-math.cos(th)), th)
        time.sleep(0.01)
