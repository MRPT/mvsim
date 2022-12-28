#!/usr/bin/env python3

# ---------------------------------------------------------------------
# This example shows how to:
# - Call MVSIM server and invoke the shutdown service to close
#   the simulation.
#
# Install python3-mvsim, or test with a local build with:
# export PYTHONPATH=$HOME/code/mvsim/build/:$PYTHONPATH
# ---------------------------------------------------------------------

from mvsim_comms import pymvsim_comms
from mvsim_msgs import SrvShutdown_pb2

import time


def call_mvsim_shutdown(client):
    # Send the request:
    req = SrvShutdown_pb2.SrvShutdown()
    # (no fields to fill in for this case)
    client.callService('shutdown', req.SerializeToString())


if __name__ == "__main__":
    client = pymvsim_comms.mvsim.Client()
    client.setName("tutorial1")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")

    call_mvsim_shutdown(client)
    print("Shutdown called.")
