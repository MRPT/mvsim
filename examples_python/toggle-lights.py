#!/usr/bin/env python3

# ---------------------------------------------------------------------
# This example shows how to:
# - Switch a light group (<light_group> XML tags) of a vehicle on and
#   off, e.g. the headlights of the robot "r1" in the warehouse demo:
#
#   mvsim launch mvsim_tutorial/demo_warehouse.world.xml
#   ./toggle-lights.py r1 headlights
#
# Install python3-mvsim, or test with a local build with:
# export PYTHONPATH=$HOME/code/mvsim/build/:$PYTHONPATH
# ---------------------------------------------------------------------

from mvsim_comms import pymvsim_comms
from mvsim_msgs import SrvGetLightState_pb2
from mvsim_msgs import SrvGetLightStateAnswer_pb2
from mvsim_msgs import SrvSetLightState_pb2
from mvsim_msgs import SrvSetLightStateAnswer_pb2

import sys
import time


def get_light_state(client, objectId, lightGroup):
    req = SrvGetLightState_pb2.SrvGetLightState()
    req.objectId = objectId
    req.lightGroup = lightGroup
    ret = client.callService('get_light_state', req.SerializeToString())
    ans = SrvGetLightStateAnswer_pb2.SrvGetLightStateAnswer()
    ans.ParseFromString(ret)
    if not ans.success:
        raise RuntimeError(ans.errorMessage)
    return ans.on


def set_light_state(client, objectId, lightGroup, on):
    req = SrvSetLightState_pb2.SrvSetLightState()
    req.objectId = objectId
    req.lightGroup = lightGroup
    req.on = on
    ret = client.callService('set_light_state', req.SerializeToString())
    ans = SrvSetLightStateAnswer_pb2.SrvSetLightStateAnswer()
    ans.ParseFromString(ret)
    if not ans.success:
        raise RuntimeError(ans.errorMessage)


if __name__ == "__main__":
    objectId = sys.argv[1] if len(sys.argv) > 1 else "r1"
    lightGroup = sys.argv[2] if len(sys.argv) > 2 else "headlights"

    client = pymvsim_comms.mvsim.Client()
    client.setName("toggle-lights")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")

    for i in range(6):
        on = not get_light_state(client, objectId, lightGroup)
        set_light_state(client, objectId, lightGroup, on)
        print(f"{objectId}/{lightGroup}: {'on' if on else 'off'}")
        time.sleep(1.0)
