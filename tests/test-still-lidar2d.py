#!/usr/bin/env python3

# ---------------------------------------------------------------------
# This example shows how to:
# - Subscribe to an MVSIM topic with a custom callback.
#
# Install python3-mvsim, or test with a local build with:
# export PYTHONPATH=$HOME/code/mvsim/build/:$PYTHONPATH
# ---------------------------------------------------------------------

from mvsim_comms import pymvsim_comms
from mvsim_msgs import ObservationLidar2D_pb2
from mvsim_msgs import SrvShutdown_pb2
from mvsim_msgs import SrvGetPose_pb2

import subprocess
import time
import os

TESTS_DIR = os.environ['TESTS_DIR']
MVSIM_CLI_EXE_PATH = os.environ['MVSIM_CLI_EXE_PATH']

global TEST_PASSED
TEST_PASSED = False


def onMessage(msgType, msg):
    global TEST_PASSED
    assert(msgType == "mvsim_msgs.ObservationLidar2D")
    p = ObservationLidar2D_pb2.ObservationLidar2D()
    p.ParseFromString(bytes(msg))
    print("callback received:\n ranges=\n" + str(p.scanRanges) +
          "\n validRanges=" + str(p.validRanges))
    scanRanges = list(p.scanRanges)
    validRanges = list(p.validRanges)

    # TODO: Fix wildly different ranges between u20.04 and u22.04!
    if (len(scanRanges) == 181) and \
        (abs(scanRanges[0]-9.0) < 1.5) and \
            (validRanges[0] == True):
        TEST_PASSED = True


def call_mvsim_shutdown(client):
    # Send the request:
    req = SrvShutdown_pb2.SrvShutdown()
    # (no fields to fill in for this case)
    client.callService('shutdown', req.SerializeToString())


if __name__ == "__main__":

    # Important: Run with simulated slow-down time (--realtime-factor),
    # to prevent timeout in real-time check asserts within mvsim
    # when run inside cloud docker containers:
    #
    subprocess.Popen([MVSIM_CLI_EXE_PATH, "launch",
                     TESTS_DIR + "/test-still-lidar2d.world.xml",
                     "--headless", "-v DEBUG", "--realtime-factor 0.1"])

    client = pymvsim_comms.mvsim.Client()
    print("Connecting to server...", flush=True)
    client.connect()
    print("Connected successfully.", flush=True)

    # Subscribe to "/r1/laser1_scan"
    client.subscribeTopic("/r1/laser1_scan", onMessage)

    for i in range(300):
        if TEST_PASSED:
            break
        print("Running and waiting...", flush=True)
        time.sleep(0.25)

    call_mvsim_shutdown(client)

    assert(TEST_PASSED)
