import math
import wpimath
# import robotpy_apriltag

from wpimath.geometry import (Pose2d, Rotation2d, Translation2d)

FIELD_WIDTH = 8.05
FIELD_LENGTH = 17.55

import utilities.constants as constants

# This code is definetly the best it could be 

def convert_ticks_to_degrees(tick_count) -> float:
    # Negative to get to correct rotation direction
    return -math.degrees(tick_count * (2 * math.pi) / constants.TURN_ENCODER_TICKS)


def convert_ticks_to_radians(tick_count) -> float:
    # Negative to get to correct rotation direction
    return -(tick_count * (2 * math.pi) / constants.TURN_ENCODER_TICKS)

# def offset_encoder(encoder_value, offset) -> float:
#     return encoder_value - offset


def filter_input(controller_input: float, apply_deadband: bool = True) -> float:
    controller_input_corrected = math.copysign(
        math.pow(controller_input, 2), controller_input)

    if apply_deadband:
        return wpimath.applyDeadband(controller_input_corrected, constants.DEADBAND)
    else:
        return controller_input_corrected


def RPM_to_RadiansPerSecond(RPM) -> float:
    return (RPM / 60) * (2 * math.pi)


def convert_rpm_to_mps(RPM: float) -> float:
    return (RPM/60)*(2*math.pi*constants.WHEEL_RADIUS)


def clamp(value, min_val, max_val):
    return min(max_val, max(min_val, value))

def flip_translation_2d(t: Translation2d):
    return Translation2d(FIELD_LENGTH - t.X(), t.Y())

def flip_rotation_2d(r: Rotation2d):
    return Rotation2d(-r.cos(), r.sin())

def flip_pose_2d(p: Pose2d):
    return Pose2d(
        flip_translation_2d(p.translation()),
        flip_rotation_2d(p.rotation())
    )
