import math

# import phoenix5
# import phoenix5.led

# Wheel Offsets
FL_OFFSET = 46.85
FR_OFFSET = 0.5
BL_OFFSET = 261.21
BR_OFFSET = 57.04

# Controller
DEADBAND = 0.15
XBOX_USB_PORT = 0
XBOX_USB_PORT_AUX = 1

MAX_LINEAR_SPEED = 10 * 0.2  # meters per sec
MAX_LINEAR_ACCELERATION = 4  # Meters per second squared

MAX_ROTATION_SPEED = 7  # radians per sec
MAX_ROTATION_ACCELERATION = 1/2  # Radians per second squared

MAX_SINGLE_SWERVE_ROTATION_SPEED = 360 * (2/3)  # degrees per sec
MAX_SINGLE_SWERVE_ROTATION_ACCELERATION = 360  # degrees per second squared

# Robot Measurements
WHEEL_RADIUS = 0.052
WHEEL_BASE = 0.55  # Center of wheel to center of wheel across the side of the bot in meters
# Center of wheel to center of wheel across the front of the bot in meters351
TRACK_WIDTH = 0.55
TURN_ENCODER_TICKS = 4096  # Ticks per revolution of the angle encoder.
GEAR_RATIO = 1 / 7.7142857


# Swerve PID constants
DRIVE_P = 2
DRIVE_I = 0
DRIVE_D = 0

TURNING_P = 2
TURNING_I = 0.0
TURNING_D = 0.1


PIGEON_CAN = 51

# Front Left
DRIVE_CAN_FL = 20
STEER_CAN_FL = 21
TURN_ENCODER_ID_FL = 19

# Front Right
DRIVE_CAN_FR = 22
STEER_CAN_FR = 23
TURN_ENCODER_ID_FR = 18

# Back Left
DRIVE_CAN_BL = 24
STEER_CAN_BL = 25
TURN_ENCODER_ID_BL = 17

# Back Right
DRIVE_CAN_BR = 26
STEER_CAN_BR = 27
TURN_ENCODER_ID_BR = 16

# Intake
INTAKE_ROLLERS_CAN = 42
INTAKE_BEAM_BREAK = 3
INTAKE_PIVOT_CAN = 31
# 40, 4
INTAKE_PIVOT_MAX_VELOCITY = 50
INTAKE_PIVOT_MAX_ACCELERATION = 4
INTAKE_PIVOT_MAX_ANGLE = 90 * math.pi / 180
INTAKE_PIVOT_MIN_ANGLE = -15 * math.pi / 180

INDEXER_CAN = 47
SHOOTER_BEAM_BREAK = 1

# Shooter
SHOOTER_PIVOT_CAN = 35
SHOOTER_PIVOT_MAX_VELOCITY = 9999
SHOOTER_PIVOT_MAX_ACCELERATION = 1000
SHOOTER_PIVOT_GEAR_RATIO = 108

SHOOTER_TOP_CAN = 37
SHOOTER_BOTTOM_CAN = 36
# Whole rotations per second (don't ask me why it's this way, I know it's bad)
MAX_SHOOTER_SPEED = 80

# LEDs
LED_CAN = 60

# BLUE_TWINKLE = phoenix5.led.SingleFadeAnimation(0, 0, 255, 0, 0.7, 66)
# GREEN_SOLID = phoenix5.led.SingleFadeAnimation(0, 255, 0, 0, 0, 66, 0)
# PURPLE_FADE = phoenix5.led.SingleFadeAnimation(255, 0, 255, 0, 0.4, 66, 0)
# RED_TWINKLE = phoenix5.led.SingleFadeAnimation(255, 0, 0, 0, 0.7, 66)
# WHITE_TWINKLE = phoenix5.led.SingleFadeAnimation(255, 255, 255, 255, 0.7, 66)
# ORANGE_FLOW = phoenix5.led.ColorFlowAnimation(
#     255, 165, 0, 0, 0.4, 66, phoenix5.led.ColorFlowAnimation.Direction.Forward)
# YELLOW_LARSON = phoenix5.led.LarsonAnimation(
#     255, 255, 0, 0, 1, 66, phoenix5.led.LarsonAnimation.BounceMode.Front, 8)
