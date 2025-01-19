import magicbot
import wpilib
import wpimath
import wpimath.kinematics

from components.chassis.drivetrain import Drivetrain, DriveSignal
import utilities.constants as constants
import utilities.ozone_utility_functions as utility_functions
import choreo
from magicbot import AutonomousStateMachine, tunable, will_reset_to



class MyRobot(magicbot.MagicRobot):
    drivetrain: Drivetrain
 

    field_oriented = True

    def createObjects(self) -> None:
        self.main_controller = wpilib.XboxController(0)


    def robotPeriodic(self) -> None:
        wpilib.SmartDashboard.putNumber("X", self.drivetrain.get_location().X())
        wpilib.SmartDashboard.putNumber("Y", self.drivetrain.get_location().Y())
        wpilib.SmartDashboard.putNumber("Rot", self.drivetrain.get_location().rotation().degrees())
        wpilib.SmartDashboard.putNumber("Left Driven", self.drivetrain.front_left.get_swerve_position().distance)
        # wpilib.SmartDashboard.putNumber("Left angle", self.drivetrain.front_left.get_swerve_position().angle)
        wpilib.SmartDashboard.putNumber("Gyro", self.drivetrain.get_yaw_value_degrees())
        

    def autonomousInit(self) -> None:
        pass
    
    def teleopInit(self) -> None:
        pass

    def autonPeriodic(self):
        pass

    def teleopPeriodic(self) -> None:
        if self.main_controller.getYButtonPressed():
            self.drivetrain.zero_gyro()
        if self.main_controller.getStartButtonPressed():
            self.cancel_all()
    
        
        if self.main_controller.getAButtonPressed():
            # self.drivetrain.enable_active_snap(90)
            print("A is pressed!!!!")

        if self.main_controller.getBButtonPressed():
            # self.drivetrain.enable_locked()
            print("B is pressed!!!!")

            

        self.drive_with_joystick(True)
        # self.tab.update()

    def disabledPeriodic(self):
        pass

    def drive_with_joystick(self, field_relative: bool) -> None:
        x_speed = -utility_functions.filter_input(
            self.main_controller.getLeftY()) * constants.MAX_LINEAR_SPEED
        y_speed = -utility_functions.filter_input(
            self.main_controller.getLeftX()) * constants.MAX_LINEAR_SPEED
        rot = -utility_functions.filter_input(
            self.main_controller.getRightX()) * constants.MAX_ROTATION_SPEED
        
        speeds = wpimath.kinematics.ChassisSpeeds(x_speed, y_speed, rot)
        self.drivetrain.set_signal(DriveSignal(speeds, field_relative))
        

    def cancel_all(self):
        pass
    

    def cancel_shooter(self):
        pass

    def cancel_intake(self):
        pass