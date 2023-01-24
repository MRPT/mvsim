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
# for i in $(seq 1 50); do bash -c "mvsim_tutorial/python/simple-obstacle-avoidance.py --vehicle veh${i} &"; done
#
# ---------------------------------------------------------------------

import argparse
import time
import math
from mvsim_comms import pymvsim_comms
from mvsim_msgs import SrvSetControllerTwist_pb2
from mvsim_msgs import ObservationLidar2D_pb2

OBS_AVOIDANCE_PERIOD = 0.1
V_MAX = 1.0
W_MAX = 1.0

parser = argparse.ArgumentParser(prog='simple-obstacle-avoidance')
parser.add_argument('--vehicle', dest='vehicleName', action='store', required=True,
                    help='The name of the vehicle to control in MVSIM')

args = parser.parse_args()

client = pymvsim_comms.mvsim.Client()

global prevLidarMsgTimestamp
prevLidarMsgTimestamp = 0


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


def evalObstacleAvoidance(obs: ObservationLidar2D_pb2.ObservationLidar2D):
    ang = -obs.aperture*0.5
    dA = obs.aperture / (len(obs.scanRanges)-1)

    # Force vectors
    resultantForce = [0, 0]

    #obsWeight = 20.0 / len(obs.scanRanges)
    obsWeight = 1

    for idx, r in enumerate(obs.scanRanges):
        ang += dA
        # obs.maxRange
        # obs.scanRanges
        # obs.sensorPose
        #
        # Obstacles:
        # Compute force strength:
        mod = min(1e3, 1.0 / r)

        # Add repulsive force:
        fx = -math.cos(ang) * mod
        fy = -math.sin(ang) * mod
        resultantForce[0] += fx * obsWeight
        resultantForce[1] += fy * obsWeight

    # Target:
    # mod = options.TARGET_ATTRACTIVE_FORCE
    # resultantForce += [cos(ang) * mod, sin(ang) * mod]

    # Result:
    desiredDirection = 0
    if (resultantForce[0] != 0) or (resultantForce[1] != 0):
        desiredDirection = math.atan2(resultantForce[1], resultantForce[0])

    # Convert direction to differential-driven command:
    v = V_MAX*math.cos(desiredDirection)
    w = W_MAX*desiredDirection/math.pi * math.copysign(1.0, v)

    return [v, w]


def onLidar2DMessage(msgType, msg):
    global prevLidarMsgTimestamp

    assert(msgType == "mvsim_msgs.ObservationLidar2D")
    p = ObservationLidar2D_pb2.ObservationLidar2D()
    p.ParseFromString(bytes(msg))
    # print("[lidar callback] received:\n ranges=\n" + str(p.scanRanges) + "\n validRanges=" + str(p.validRanges))

    if (p.unixTimestamp > prevLidarMsgTimestamp+OBS_AVOIDANCE_PERIOD):
        prevLidarMsgTimestamp = p.unixTimestamp
        [v, w] = evalObstacleAvoidance(p)
        sendRobotTwistSetpoint(client, args.vehicleName, v, 0, w)


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

    time.sleep(20)
    sendRobotTwistSetpoint(client, args.vehicleName, 0, 0, 0)
