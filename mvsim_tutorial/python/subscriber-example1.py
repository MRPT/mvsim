#!/usr/bin/env python3

# To test with a local build:
#
# export PYTHONPATH=$HOME/code/mvsim/build/lib/:$PYTHONPATH

from mvsim_comms import pymvsim_comms
import time

if __name__ == "__main__":
    client = pymvsim_comms.mvsim.Client()
    client.setName("tutorial1")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")

    # Subscribe to "/r1/pose"
    # client.subscribeTopic()

    time.sleep(2.0)
