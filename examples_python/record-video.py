#!/usr/bin/env python3

# ---------------------------------------------------------------------
# This example shows how to:
# - Record a video of the 3D view of a running simulation, e.g.:
#
#   mvsim launch mvsim_tutorial/demo_suspension.world.xml
#   ./record-video.py --output truck.mp4 --duration 10 --follow r1
#
#   The video is in simulation time: it lasts "duration" seconds of
#   simulated time, and plays at real speed even if the simulation is
#   slower or faster than real time. Requires ffmpeg, unless the output
#   is an image sequence (e.g. "frames/img_%06d.png").
#
# Install python3-mvsim, or test with a local build with:
# export PYTHONPATH=$HOME/code/mvsim/build/:$PYTHONPATH
# ---------------------------------------------------------------------

from mvsim_comms import pymvsim_comms
from mvsim_msgs import SrvStartVideoRecording_pb2
from mvsim_msgs import SrvStartVideoRecordingAnswer_pb2
from mvsim_msgs import SrvStopVideoRecording_pb2
from mvsim_msgs import SrvStopVideoRecordingAnswer_pb2
from mvsim_msgs import SrvGetVideoRecordingStatus_pb2
from mvsim_msgs import SrvGetVideoRecordingStatusAnswer_pb2

import argparse
import time


def start_recording(client, args):
    req = SrvStartVideoRecording_pb2.SrvStartVideoRecording()
    if args.output:
        req.outputFile = args.output
    req.fps = args.fps
    req.width = args.width
    req.height = args.height
    req.duration = args.duration
    if args.follow:
        # An orbit camera looking at a vehicle (otherwise, the GUI view):
        req.camera = "orbit"
        req.followVehicle = args.follow
        req.followHeading = True
        req.camDistance = args.distance
        req.camAzimuthDeg = args.azimuth
        req.camElevationDeg = args.elevation
    ret = client.callService('start_video_recording', req.SerializeToString())
    ans = SrvStartVideoRecordingAnswer_pb2.SrvStartVideoRecordingAnswer()
    ans.ParseFromString(ret)
    if not ans.success:
        raise RuntimeError(ans.errorMessage)
    return ans.outputFile


def get_status(client):
    req = SrvGetVideoRecordingStatus_pb2.SrvGetVideoRecordingStatus()
    ret = client.callService(
        'get_video_recording_status', req.SerializeToString())
    ans = SrvGetVideoRecordingStatusAnswer_pb2.SrvGetVideoRecordingStatusAnswer()
    ans.ParseFromString(ret)
    return ans


def stop_recording(client):
    req = SrvStopVideoRecording_pb2.SrvStopVideoRecording()
    ret = client.callService('stop_video_recording', req.SerializeToString())
    ans = SrvStopVideoRecordingAnswer_pb2.SrvStopVideoRecordingAnswer()
    ans.ParseFromString(ret)
    return ans


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Records a video of a running MVSim simulation")
    parser.add_argument("--output", default="",
                        help="Output file (default: mvsim_video_<date>.mp4)")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Simulated time to record [s]")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--follow", default="",
                        help="Vehicle to follow (default: the GUI view)")
    parser.add_argument("--distance", type=float, default=12.0)
    parser.add_argument("--azimuth", type=float, default=-120.0,
                        help="Relative to the vehicle heading [deg]")
    parser.add_argument("--elevation", type=float, default=15.0)
    args = parser.parse_args()

    client = pymvsim_comms.mvsim.Client()
    client.setName("record-video")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")

    print(f"Recording to: {start_recording(client, args)}")

    # It stops by itself after "duration" seconds of simulated time:
    try:
        while True:
            st = get_status(client)
            if not st.recording:
                break
            print(f"\r{st.frames} frames, {st.duration:.1f} s", end="")
            time.sleep(0.5)
    except KeyboardInterrupt:
        ans = stop_recording(client)
        if not ans.success:
            print(f"\nError: {ans.errorMessage}")

    st = get_status(client)
    if st.errorMessage:
        print(f"\nError: {st.errorMessage}")
    print("\nDone.")
