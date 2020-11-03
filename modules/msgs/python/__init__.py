import sys, os

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'mvsim_msgs'))

from . import Pose_pb2
from . import SrvSetPose_pb2

del sys.path[0], sys, os
