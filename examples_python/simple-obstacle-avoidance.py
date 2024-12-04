#!/usr/bin/env python3

# ---------------------------------------------------------------------
# This example shows how to:
# - Call a MVSIM service.
# - Use it to drive a vehicle/robot using its internal kinematics
#   controller.
# - Subscribe to 2D lidar data
#
# Install python3-mvsim, or test with a local build with:
# export PYTHONPATH=$HOME/code/mvsim/build/:$PYTHONPATH
#
# Demo with N robots:
# for i in $(seq 1 25); do bash -c "examples_python/simple-obstacle-avoidance.py --vehicle veh${i} &"; done
#
# ---------------------------------------------------------------------

import argparse
import time
import math
import random
from mvsim_comms import pymvsim_comms
from mvsim_msgs import SrvSetControllerTwist_pb2
from mvsim_msgs import ObservationLidar2D_pb2
from mvsim_msgs import SrvGetPose_pb2, SrvGetPoseAnswer_pb2

OBS_AVOIDANCE_PERIOD = 0.2  # s
V_MAX = 1.0  # m/s
VIRTUAL_TARGET_DIST = 2.0  # m
NEW_TARGET_PERIOD_SECONDS = 10  # s
RANDOM_TARGET_RANGE = 30
TARGET_ATTRACTIVE_FORCE = 10

parser = argparse.ArgumentParser(prog='simple-obstacle-avoidance')
parser.add_argument('--vehicle', dest='vehicleName', action='store', required=True,
                    help='The name of the vehicle to control in MVSIM')

args = parser.parse_args()

client = pymvsim_comms.mvsim.Client()
random.seed()

global prevLidarMsgTimestamp
prevLidarMsgTimestamp = 0

global prevGlobalGoalTimestamp
global prevGlobalGoal
prevGlobalGoal = [0, 0]
prevGlobalGoalTimestamp = 0


def sendRobotTwistSetpoint(client, robotName, vx, vy, w):
    # (vx,vy) in local coordinates [m/s]
    # (w) in [rad/s]

    req = SrvSetControllerTwist_pb2.SrvSetControllerTwist()
    req.objectId = robotName  # vehicle/robot/object name in MVSIM
    req.twistSetPoint.vx = vx
    req.twistSetPoint.vy = vy
    req.twistSetPoint.vz = 0
    req.twistSetPoint.wx = 0
    req.twistSetPoint.wy = 0
    req.twistSetPoint.wz = w
    # ret =
    client.callService('set_controller_twist', req.SerializeToString())


def getRobotPose(client, robotName):
    req = SrvGetPose_pb2.SrvGetPose()
    req.objectId = robotName  # vehicle/robot/object name in MVSIM
    ret = client.callService('get_pose', req.SerializeToString())
    ans = SrvGetPoseAnswer_pb2.SrvGetPoseAnswer()
    ans.ParseFromString(ret)
    print(ans)


def evalObstacleAvoidance(obs: ObservationLidar2D_pb2.ObservationLidar2D):
    ang = -obs.aperture*0.5
    dA = obs.aperture / (len(obs.scanRanges)-1)

    # Force vectors
    resultantForce = [0, 0]

    # obsWeight = 20.0 / len(obs.scanRanges)
    obsWeight = 1

    for idx, r in enumerate(obs.scanRanges):
        ang += dA
        # obs.maxRange
        # obs.scanRanges
        # obs.sensorPose
        #
        # Obstacles:
        # Compute force strength:
        mod = min(1e3, 1.0 / (r*r))

        # Add repulsive force:
        fx = -math.cos(ang) * mod
        fy = -math.sin(ang) * mod
        resultantForce[0] += fx * obsWeight
        resultantForce[1] += fy * obsWeight

    # Target:
    # robotPose = getRobotPose(client, args.vehicleName)
    # TODO: Compute relative vector
    ang = 0
    resultantForce[0] += math.cos(ang) * TARGET_ATTRACTIVE_FORCE
    resultantForce[1] += math.sin(ang) * TARGET_ATTRACTIVE_FORCE

    # Result:
    desiredDirection = 0
    if (resultantForce[0] != 0) or (resultantForce[1] != 0):
        desiredDirection = math.atan2(resultantForce[1], resultantForce[0])

    # Convert direction to differential-driven command:
    # Build a "virtual target point" and calculate the (v,w) to drive there.
    target_x = VIRTUAL_TARGET_DIST*math.cos(desiredDirection)
    target_y = VIRTUAL_TARGET_DIST*math.sin(desiredDirection)

    if (target_x < 0):
        v = -V_MAX
    else:
        v = V_MAX

    if (target_y == 0):
        w = 0
    else:
        R = math.sqrt((target_x*target_x + target_y *
                      target_y)/(2*abs(target_y)))
        if (target_y < 0):
            R = -R
        w = v / R

    return [v, w]


def onLidar2DMessage(msgType, msg):
    global prevLidarMsgTimestamp
    global prevGlobalGoalTimestamp, prevGlobalGoal

    assert (msgType == "mvsim_msgs.ObservationLidar2D")
    p = ObservationLidar2D_pb2.ObservationLidar2D()
    p.ParseFromString(bytes(msg))
    # print("[lidar callback] received:\n ranges=\n" + str(p.scanRanges) + "\n validRanges=" + str(p.validRanges))

    if (p.unixTimestamp > prevLidarMsgTimestamp+OBS_AVOIDANCE_PERIOD):
        prevLidarMsgTimestamp = p.unixTimestamp
        [v, w] = evalObstacleAvoidance(p)
        sendRobotTwistSetpoint(client, args.vehicleName, v, 0, w)

    if (p.unixTimestamp > prevGlobalGoalTimestamp+NEW_TARGET_PERIOD_SECONDS):
        prevGlobalGoalTimestamp = p.unixTimestamp
        prevGlobalGoal = [(-0.5+random.random())*RANDOM_TARGET_RANGE,
                          (-0.5+random.random())*RANDOM_TARGET_RANGE]


if __name__ == "__main__":
    client.setName("obstacle-avoidance")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")

    print("===========================")
    print("Configuration       ")
    print("- Vehicle to control: " + args.vehicleName)
    print("===========================")

    # Subscribe to "/r1/laser1_scan"
    client.subscribeTopic("/" + args.vehicleName +
                          "/laser1_scan", onLidar2DMessage)

    time.sleep(60)
    sendRobotTwistSetpoint(client, args.vehicleName, 0, 0, 0)
