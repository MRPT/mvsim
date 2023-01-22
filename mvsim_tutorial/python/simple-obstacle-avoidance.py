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
# ---------------------------------------------------------------------

import argparse
import time
from mvsim_comms import pymvsim_comms
from mvsim_msgs import SrvSetControllerTwist_pb2
from mvsim_msgs import ObservationLidar2D_pb2

OBS_AVOIDANCE_PERIOD = 0.2

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

    for idx, r in enumerate(obs.scanRanges):
        print(idx, r, ang)
        ang += dA
    # obs.maxRange
    # obs.scanRanges
    # obs.sensorPose
    #


def onLidar2DMessage(msgType, msg):
    global prevLidarMsgTimestamp

    assert(msgType == "mvsim_msgs.ObservationLidar2D")
    p = ObservationLidar2D_pb2.ObservationLidar2D()
    p.ParseFromString(bytes(msg))
    #print("[lidar callback] received:\n ranges=\n" + str(p.scanRanges) + "\n validRanges=" + str(p.validRanges))

    if (p.unixTimestamp > prevLidarMsgTimestamp+OBS_AVOIDANCE_PERIOD):
        prevLidarMsgTimestamp = p.unixTimestamp
        evalObstacleAvoidance(p)


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

    sendRobotTwistSetpoint(client, args.vehicleName, 0.75, 0.0, 1.0)

    time.sleep(10)
