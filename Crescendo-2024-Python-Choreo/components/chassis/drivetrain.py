from phoenix6 import hardware, configs, signals
import wpimath.controller
import wpimath.estimator
import wpimath.geometry
import wpimath.kinematics
import wpilib
import math

import utilities.constants as constants
import components.chassis.swervemodule as swervemodule

from choreo import SwerveSample

class Drivetrain:
    instance = None

    MAX_SPEED = constants.MAX_LINEAR_SPEED
    MAX_ANGULAR_SPEED = constants.MAX_ROTATION_SPEED

    def __init__(self) -> None:
        wheel_base = constants.WHEEL_BASE
        track_width = constants.TRACK_WIDTH
        
        
        self.x_controller = wpimath.controller.PIDController(5, 0.0, 0.6)
        self.y_controller = wpimath.controller.PIDController(5, 0.0, 0.6)
        self.heading_controller = wpimath.controller.PIDController(2.0, 0.0, 0.3)
        self.heading_controller.setTolerance(0.2)
        self.heading_controller.enableContinuousInput(0, math.tau)

        self.front_left_location = wpimath.geometry.Translation2d(
            wheel_base/2, track_width/2)
        self.front_right_location = wpimath.geometry.Translation2d(
            wheel_base/2, -track_width/2)
        self.back_left_location = wpimath.geometry.Translation2d(
            -wheel_base/
            2, track_width/2)
        self.back_right_location = wpimath.geometry.Translation2d(
            -wheel_base/2, -track_width/2)

        self.front_left = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_FL,
            turning_motor_id=constants.STEER_CAN_FL,
            turning_encoder_id=constants.TURN_ENCODER_ID_FL,
            offset=constants.FL_OFFSET,
            name="Front Left"
        )

        self.front_right = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_FR,
            turning_motor_id=constants.STEER_CAN_FR,
            turning_encoder_id=constants.TURN_ENCODER_ID_FR,
            offset=constants.FR_OFFSET,
            name="Front Right"
        )

        self.back_left = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_BL,
            turning_motor_id=constants.STEER_CAN_BL,
            turning_encoder_id=constants.TURN_ENCODER_ID_BL,
            offset=constants.BL_OFFSET,
            name="Back Left"
        )

        self.back_right = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_BR,
            turning_motor_id=constants.STEER_CAN_BR,
            turning_encoder_id=constants.TURN_ENCODER_ID_BR,
            offset=constants.BR_OFFSET,
            name="Back Right"
        )

        self.gyro = hardware.Pigeon2(constants.PIGEON_CAN, "*")

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.front_left_location,
            self.front_right_location,
            self.back_left_location,
            self.back_right_location
        )

        self.pose_estimator = wpimath.estimator.SwerveDrive4PoseEstimator(
            self.kinematics,
            self.get_yaw_value(),
            (
                self.front_left.get_swerve_position(),
                self.front_right.get_swerve_position(),
                self.back_left.get_swerve_position(),
                self.back_right.get_swerve_position()
            ),
            wpimath.geometry.Pose2d()
        )

        
        self.snap_angle_pid = wpimath.controller.PIDController(4, 0, 0)
        self.snap_angle_pid.enableContinuousInput(0, math.tau)
        self.snap_angle_pid.setTolerance(math.radians(3))

        self.zero_gyro()
        
        self.timer = wpilib.Timer()

        self.field_relative = True

        self.signal = None
        self.snap_angle = None
        self.passive_snap_angle = None

        self.active_snap_enabled = False
        self.passive_snap_enabled = False
        self.is_locked = False

        Drivetrain.instance = self


    def get_yaw_value(self) -> wpimath.geometry.Rotation2d:
        return wpimath.geometry.Rotation2d(math.radians(self.gyro.get_yaw().value))
    
    def get_yaw_value_degrees(self) -> float:
        return self.get_yaw_value().degrees()

    def get_signal(self):
        return self.signal
    
    def set_signal(self, signal):
        if(isinstance(signal, DriveSignal)):
            self.signal = signal

    def drive(
        self,
        speeds: wpimath.kinematics.ChassisSpeeds,
        field_relative: bool
    ) -> None:

        location = self.get_location()

        target_angle = None
        target_states = None
        last_angular_input = 0

        if(abs(speeds.omega) > 1e-5): #Checking if bot is rotating
            last_angular_input = self.timer.getFPGATimestamp()

        if self.active_snap_enabled:       
            if self.snap_angle != None:
                target_angle = self.snap_angle
        # else: 
        #     if(not wpilib.DriverStation.isAutonomous() and self.timer.getFPGATimestamp() - last_angular_input >= 0.4):
        #         if not self.passive_snap_enabled and not wpilib.DriverStation.isAutonomous():
        #             self.enable_passive_snap()
        #         if self.passive_snap_angle is not None:
        #             target_angle = self.passive_snap_angle
        #     elif(self.passive_snap_enabled):
        #         self.disable_passive_snap()
        
        if self.is_locked:    
            target_states = (wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d.fromDegrees(225)),
                             wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d.fromDegrees(135)),
                             wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d.fromDegrees(315)),
                             wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d.fromDegrees(225))
                             )
            target_angle = None

        if target_angle != None:
            self.snap_angle_pid.setSetpoint(target_angle.radians())
            speeds.omega = self.snap_angle_pid.calculate(location.rotation().radians())
            if self.snap_angle_pid.atSetpoint():
                target_angle = None

        if field_relative:
            speeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(speeds, self.get_yaw_value())

        if not self.is_locked:
            target_states = self.kinematics.toSwerveModuleStates(speeds)
        
        target_states = self.kinematics.desaturateWheelSpeeds(target_states, Drivetrain.MAX_SPEED)

        self.front_left.set_desired_state(target_states[0])
        self.front_right.set_desired_state(target_states[1])
        self.back_left.set_desired_state(target_states[2])
        self.back_right.set_desired_state(target_states[3])

    def enable_active_snap(self, angle):
        self.disable_passive_snap()
        self.snap_angle = wpimath.geometry.Rotation2d(angle)
        self.active_snap_enabled = True
        self.snap_angle_pid.reset()
        print(f"snapping to {angle} degrees")


    def enable_passive_snap(self):
        self.passive_snap_enabled = True
        self.passive_snap_angle = self.get_location().rotation()
     
    def enable_locked(self):
        self.is_locked = True
    
    def disable_active_snap(self):
        self.active_snap_enabled = False
        self.snap_angle = None

    def disable_passive_snap(self):
        self.passive_snap_enabled = False
        self.passive_snap_angle = False

    def disable_locked(self):
        self.is_locked = False

    def set_field_relative(self, field_relative) -> bool:
        if(isinstance(field_relative, bool)):
            self.field_relative = field_relative

    def is_field_relative(self) -> bool:
        return self.field_relative  

    def apply_voltage(self, voltage): #for testing purposes
        self.back_left.set_module_voltage(voltage, voltage)
        self.back_right.set_module_voltage(voltage, voltage)
        self.front_left.set_module_voltage(voltage, voltage)
        self.front_right.set_module_voltage(voltage, voltage)

    def get_location(self) -> wpimath.geometry.Pose2d:
        return self.pose_estimator.getEstimatedPosition()
    
    def get_module_postitions(self) -> wpimath.kinematics.SwerveModuleState:
        return (self.front_left.get_swerve_position(),
                self.front_right.get_swerve_position(),
                self.back_left.get_swerve_position(),
                self.back_right.get_swerve_position()
                )

    def zero_gyro(self) -> None:
        self.gyro.set_yaw(0)

    def execute(self) -> None:
        self.pose_estimator.update(self.get_yaw_value(), 
                                   self.get_module_postitions())
        
        
        if self.signal is not None:
            self.drive(self.signal.get_speed(), self.signal.get_field_relative())
            
        # print(self.get_location().X(), self.get_location().Y(), self.get_location().rotation().radians())
        
       
    def follow_trajectory(self, sample: SwerveSample):
        if not sample:
            print("No valid trajectory sample provided.")
            return      


        # Get the current pose of the robot
        pose = self.get_location()
        # print(sample.omega)

        # Generate the next speeds for the robot
        speeds = wpimath.kinematics.ChassisSpeeds(
            sample.vx + self.x_controller.calculate(pose.X(), sample.x),
            sample.vy + self.y_controller.calculate(pose.Y(), sample.y),
            sample.omega + self.heading_controller.calculate(pose.rotation().radians(), sample.heading)
        )
        
        # print(sample.vx + self.x_controller.calculate(pose.X(), sample.x),
        #       sample.vy + self.y_controller.calculate(pose.Y(), sample.y),
        #       sample.omega + self.heading_controller.calculate(pose.rotation().radians(), sample.heading)
        #     )

        # Apply the generated speed
        print(speeds)
        self.set_signal(DriveSignal(speeds, True))
        
    def reset_odometry(self, loc: wpimath.geometry.Pose2d) -> None:
        self.pose_estimator.resetPosition(self.get_yaw_value(),
                                          self.get_module_postitions(),
                                          self.get_location())
        self.pose_estimator.resetPose(loc)
        

class DriveSignal():
    def __init__(self, speed: wpimath.kinematics.ChassisSpeeds, field_relative: bool):
        self._speed = speed
        self._field_relative = field_relative

    def get_speed(self):
        return self._speed

    def get_field_relative(self):
        return self._field_relative