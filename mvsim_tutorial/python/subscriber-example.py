#!/usr/bin/env python3

# ---------------------------------------------------------------------
# This example shows how to:
# - Subscribe to an MVSIM topic with a custom callback.
#
# Install python3-mvsim, or test with a local build with:
# export PYTHONPATH=$HOME/code/mvsim/build/:$PYTHONPATH
# ---------------------------------------------------------------------

from mvsim_comms import pymvsim_comms
from mvsim_comms import pymvsim_comms
import time


# Callback for subscribed topic:
def onMessage(msg):
    print("callback received: " + msg)


if __name__ == "__main__":
    client = pymvsim_comms.mvsim.Client()
    client.setName("tutorial1")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")

    # Subscribe to "/r1/pose"
    client.subscribeTopic("/r1/pose", onMessage)

    time.sleep(2.0)
