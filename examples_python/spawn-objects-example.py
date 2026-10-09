#!/usr/bin/env python3

# ---------------------------------------------------------------------
# This example shows how to:
# - Spawn, move and remove visual-only objects at runtime (e.g. marks
#   drawn on the ground, targets), visible to camera sensors.
# - Read the ground truth pose of all objects.
#
# Install python3-mvsim, or test with a local build with:
# export PYTHONPATH=$HOME/code/mvsim/build/:$PYTHONPATH
# ---------------------------------------------------------------------

from mvsim_comms import pymvsim_comms
from mvsim_msgs import SrvSpawnObjects_pb2, SrvSpawnObjectsAnswer_pb2
from mvsim_msgs import SrvRemoveObjects_pb2, SrvRemoveObjectsAnswer_pb2
from mvsim_msgs import SrvGetAllPoses_pb2, SrvGetAllPosesAnswer_pb2
from mvsim_msgs import RuntimeObject_pb2
import math
import time


def spawn_ground_marks(client, n=100, radius=3.0):
    """Spawns a circle of small round marks lying on the ground (one call)"""
    req = SrvSpawnObjects_pb2.SrvSpawnObjects()
    for i in range(n):
        o = req.objects.add()
        o.name = f'marks/{i}'
        o.shape = RuntimeObject_pb2.RuntimeObject.DISK
        th = 2 * math.pi * i / n
        o.pose.x = radius * math.cos(th)
        o.pose.y = radius * math.sin(th)
        o.pose.z = 0.003  # height over the ground
        o.pose.yaw = o.pose.pitch = o.pose.roll = 0
        o.sizeX = 0.1  # diameter
        o.color = 0xffffffff  # RGBA
        o.onGround = True

    # A box target:
    o = req.objects.add()
    o.name = 'target'
    o.shape = RuntimeObject_pb2.RuntimeObject.BOX
    o.pose.x, o.pose.y, o.pose.z = 2.0, 0.0, 0.25
    o.pose.yaw = o.pose.pitch = o.pose.roll = 0
    o.sizeX = o.sizeY = o.sizeZ = 0.5
    o.color = 0x00ff00ff

    ans = SrvSpawnObjectsAnswer_pb2.SrvSpawnObjectsAnswer()
    ans.ParseFromString(client.callService('spawn_objects', req.SerializeToString()))
    print(f'spawn_objects: success={ans.success} {ans.errorMessage}')


def remove_objects(client, prefix):
    req = SrvRemoveObjects_pb2.SrvRemoveObjects()
    req.prefix = prefix
    ans = SrvRemoveObjectsAnswer_pb2.SrvRemoveObjectsAnswer()
    ans.ParseFromString(client.callService('remove_objects', req.SerializeToString()))
    print(f'remove_objects: removed {ans.numRemoved} objects')


def print_all_poses(client):
    req = SrvGetAllPoses_pb2.SrvGetAllPoses()
    ans = SrvGetAllPosesAnswer_pb2.SrvGetAllPosesAnswer()
    ans.ParseFromString(client.callService('get_all_poses', req.SerializeToString()))
    print(f'get_all_poses: t={ans.simulTime:.3f} s, {len(ans.objects)} objects')
    for o in ans.objects[:5]:
        print(f'  {o.name}: x={o.pose.x:.3f} y={o.pose.y:.3f} yaw={o.pose.yaw:.3f}')


if __name__ == "__main__":
    client = pymvsim_comms.mvsim.Client()
    client.setName("spawn-objects-example")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")

    spawn_ground_marks(client)
    print_all_poses(client)
    time.sleep(2.0)
    remove_objects(client, 'marks/')
    print_all_poses(client)
