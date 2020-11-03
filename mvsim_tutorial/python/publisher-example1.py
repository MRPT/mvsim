#!/usr/bin/env python3

# To test with a local build:
#
# export PYTHONPATH=$HOME/code/mvsim/build/lib/:$PYTHONPATH
# export PYTHONPATH=/tmp/install-mvsim/include/mvsim/:$PYTHONPATH

import pymvsim_comms
from mvsim_msgs import SrvSetPose_pb2
import time
from math import radians

if __name__ == "__main__":
    client = pymvsim_comms.mvsim.Client()
    client.setName("tutorial1")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")

    # Send a set pose request:
    req = SrvSetPose_pb2.SrvSetPose()
    req.objectId = 'veh1'  # vehicle/robot/object name in MVSIM
    req.pose.x = 1.0
    req.pose.y = 1.0
    req.pose.z = 1.0
    req.pose.yaw = radians(0.0)
    req.pose.pitch = radians(0.0)
    req.pose.roll = radians(0.0)
    print(req)
    s = req.SerializeToString()
    print(s)

    time.sleep(2.0)
