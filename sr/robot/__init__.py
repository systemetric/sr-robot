# This log import configures our logging for us, but we don't want to
# provide it as part of this package.
from sr.robot import log as _log

from sr.robot.robot import Robot, NoCameraPresent
from sr.robot.power import OUT_H0, OUT_H1, OUT_L0, OUT_L1, OUT_L2, OUT_L3
from sr.robot.vision import MARKER_ARENA, MARKER_TOKEN, MARKER_BUCKET_SIDE, MARKER_BUCKET_END
from sr.robot.ruggeduino import INPUT, OUTPUT, INPUT_PULLUP, Ruggeduino

__all__ = [
    "Robot", "NoCameraPresent",
    "OUT_H0", "OUT_H1", "OUT_L0", "OUT_L1", "OUT_L2", "OUT_L3",
    "MARKER_ARENA", "MARKER_TOKEN", "MARKER_BUCKET_SIDE", "MARKER_BUCKET_END",
    "INPUT", "OUTPUT", "INPUT_PULLUP", "Ruggeduino",
]
