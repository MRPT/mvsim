#!/usr/bin/env python3

# ---------------------------------------------------------------------
# This example shows how to:
# - Subscribe to an MVSIM topic with a custom callback.
#
# Install python3-mvsim, or test with a local build with:
# export PYTHONPATH=$HOME/code/mvsim/build/:$PYTHONPATH
# ---------------------------------------------------------------------

from mvsim_comms import pymvsim_comms
from mvsim_msgs import TimeStampedPose_pb2
from mvsim_msgs import ObservationLidar2D_pb2
import time


# Callback for subscribed topic:
def onPoseMessage(msgType, msg):
    assert(msgType == "mvsim_msgs.TimeStampedPose")
    p = TimeStampedPose_pb2.TimeStampedPose()
    p.ParseFromString(bytes(msg))
    print("[pose callback] received: pose=\n" + str(p))


def onLidar2DMessage(msgType, msg):
    assert(msgType == "mvsim_msgs.ObservationLidar2D")
    p = ObservationLidar2D_pb2.ObservationLidar2D()
    p.ParseFromString(bytes(msg))
    print("[lidar callback] received:\n ranges=\n" +
          str(p.scanRanges) + "\n validRanges=" + str(p.validRanges))


if __name__ == "__main__":
    client = pymvsim_comms.mvsim.Client()
    client.setName("tutorial1")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")

    # Subscribe to "/r1/pose"
    client.subscribeTopic("/r1/pose", onPoseMessage)

    # Subscribe to "/r1/laser1_scan"
    client.subscribeTopic("/r1/laser1_scan", onLidar2DMessage)

    time.sleep(2.0)
