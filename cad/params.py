"""The leader arm's numbers.  It is the RobotDoctor arm at half scale, joint for joint, so that moving it moves the
big arm the same way and the operator's hand traces the tool's path at half size.

Big-arm numbers are quoted from ~/Desktop/RobotDoctor/cad/arm_concept.py (09-05: L_UPPER 320, L_FORE 400, J2_H 130).
"""
from __future__ import annotations

from cad.servos import XL330_M288

SERVO = XL330_M288
SCALE = 0.5

# the arm's kinematic chain, then ours
ARM = dict(J2_H=130.0, L_UPPER=320.0, L_FORE=400.0, W5_Z=55.0)
J2_H = ARM["J2_H"] * SCALE          # 65: J2 axis above the base's J1 output face
L_UPPER = ARM["L_UPPER"] * SCALE    # 160: J2 -> J3
L_FORE = ARM["L_FORE"] * SCALE      # 200: J3 -> wrist centre (J5 axis)
HANDLE_L = 90.0                     # J6's handle: the operator's grip, along the tool axis
TRIGGER_TRAVEL_DEG = 40.0           # the 7th servo's travel on the trigger (the gripper's open/close)

# joint order and axes, the arm's: J1 yaw (Z), J2 pitch (X), J3 pitch (X), J4 roll (link Z), J5 pitch (X), J6 roll (link Z)
JOINTS = ("j1", "j2", "j3", "j4", "j5", "j6", "trigger")
RANGES = {"j1": (-160, 160), "j2": (-8, 178), "j3": (-160, 160), "j4": (-180, 180), "j5": (-100, 100), "j6": (-180, 180),
          "trigger": (0, TRIGGER_TRAVEL_DEG)}      # the arm's own (STOP_ANGLES, J1_RANGE); J5 is the hand's band

# printed links
LINK_W = 24.0                       # link bar width (fits the servo's 20 mm case with 2 mm walls)
LINK_T = 12.0                       # link bar thickness
WALL = 2.0
BASE_RISER = 100.0                  # raises the default grip clear of a full desktop
BASE_D = 120.0                      # base disc, clamps or screws to the desk
COUNTERBALANCE = True               # a rubber band from the base post to the upper arm: the servos hold nothing when idle
