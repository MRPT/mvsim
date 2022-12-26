#!/usr/bin/env python3

# ---------------------------------------------------------------------
# This example shows how to:
# - Subscribe to an MVSIM topic with a custom callback.
#
# Install python3-mvsim, or test with a local build with:
# export PYTHONPATH=$HOME/code/mvsim/build/:$PYTHONPATH
# ---------------------------------------------------------------------

from mvsim_comms import pymvsim_comms
from mvsim_msgs import GenericObservation_pb2

import subprocess
import time
import os

TESTS_DIR = os.environ['TESTS_DIR']
MVSIM_CLI_EXE_PATH = os.environ['MVSIM_CLI_EXE_PATH']


def onMessage(msgType, msg):
    assert(msgType == "mvsim_msgs.GenericObservation")
    p = GenericObservation_pb2.GenericObservation()
    p.ParseFromString(bytes(msg))
    print("callback received: obs=\n" + str(p))


if __name__ == "__main__":

    subprocess.Popen([MVSIM_CLI_EXE_PATH, "launch",
                     TESTS_DIR + "/test-still-lidar2d.world.xml",
                     "--headless", "-v WARN"])

    client = pymvsim_comms.mvsim.Client()
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")

    # Subscribe to "/r1/laser1"
    client.subscribeTopic("/r1/laser1", onMessage)

    time.sleep(120.0)
