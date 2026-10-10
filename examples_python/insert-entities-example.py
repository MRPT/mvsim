#!/usr/bin/env python3

# ---------------------------------------------------------------------
# This example shows how to insert entities (robots, obstacles, world
# elements) into a running simulation, given as world XML, and how to
# remove them later.
#
# Start a simulation first, e.g.:
#   mvsim launch mvsim_tutorial/demo_1robot.world.xml
#
# Install python3-mvsim, or test with a local build with:
# export PYTHONPATH=$HOME/code/mvsim/build/:$PYTHONPATH
# ---------------------------------------------------------------------

from mvsim_comms import pymvsim_comms
from mvsim_msgs import SrvInsertEntities_pb2, SrvInsertEntitiesAnswer_pb2
from mvsim_msgs import SrvRemoveEntities_pb2, SrvRemoveEntitiesAnswer_pb2
from mvsim_msgs import SrvSetControllerTwist_pb2
from mvsim_msgs import SrvGetAllPoses_pb2, SrvGetAllPosesAnswer_pb2
import time

# Same tags as in a world file. Relative paths are resolved wrt the directory
# of the world file loaded by the simulator.
NEW_ROBOT_XML = """
<include file="../definitions/jackal.vehicle.xml" default_sensors="false" />
<vehicle name="spawned_robot" class="jackal">
  <init_pose>2 2 0</init_pose>
</vehicle>
"""

BOX_XML = """
<block name="box_{i}">
  <static>true</static>
  <zmin>0</zmin> <zmax>0.5</zmax>
  <color>#a06020</color>
  <shape><pt>-0.25 -0.25</pt><pt>0.25 -0.25</pt><pt>0.25 0.25</pt><pt>-0.25 0.25</pt></shape>
  <init_pose>{x} -2 0</init_pose>
</block>
"""


def insert_entities(client, xml):
    req = SrvInsertEntities_pb2.SrvInsertEntities()
    req.xml = xml
    ans = SrvInsertEntitiesAnswer_pb2.SrvInsertEntitiesAnswer()
    ans.ParseFromString(client.callService('insert_entities', req.SerializeToString()))
    if not ans.success:
        raise RuntimeError(f'insert_entities failed: {ans.errorMessage}')
    print(f'insert_entities: {list(ans.names)}')
    return list(ans.names)


def remove_entities(client, names):
    req = SrvRemoveEntities_pb2.SrvRemoveEntities()
    req.names.extend(names)
    ans = SrvRemoveEntitiesAnswer_pb2.SrvRemoveEntitiesAnswer()
    ans.ParseFromString(client.callService('remove_entities', req.SerializeToString()))
    print(f'remove_entities: removed {ans.numRemoved} {ans.errorMessage}')


def set_twist(client, robot, vx, w):
    req = SrvSetControllerTwist_pb2.SrvSetControllerTwist()
    req.objectId = robot
    req.twistSetPoint.vx = vx
    req.twistSetPoint.vy = 0
    req.twistSetPoint.vz = 0
    req.twistSetPoint.wx = 0
    req.twistSetPoint.wy = 0
    req.twistSetPoint.wz = w
    client.callService('set_controller_twist', req.SerializeToString())


def print_pose(client, name):
    req = SrvGetAllPoses_pb2.SrvGetAllPoses()
    req.prefix = name
    ans = SrvGetAllPosesAnswer_pb2.SrvGetAllPosesAnswer()
    ans.ParseFromString(client.callService('get_all_poses', req.SerializeToString()))
    for o in ans.objects:
        print(f'  {o.name}: x={o.pose.x:.2f} y={o.pose.y:.2f} yaw={o.pose.yaw:.2f}')


if __name__ == "__main__":
    client = pymvsim_comms.mvsim.Client()
    client.setName("insert-entities-example")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")

    robot = insert_entities(client, NEW_ROBOT_XML)
    boxes = insert_entities(client, ''.join(
        BOX_XML.format(i=i, x=1.0 + i) for i in range(5)))

    # Drive the new robot for a while:
    for _ in range(30):
        set_twist(client, 'spawned_robot', 0.5, 0.3)
        time.sleep(0.1)
    set_twist(client, 'spawned_robot', 0, 0)
    print_pose(client, 'spawned_robot')

    time.sleep(2.0)
    remove_entities(client, robot + boxes)
