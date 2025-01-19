from magicbot import AutonomousStateMachine, timed_state, state
from wpilib import DriverStation
from components.chassis.drivetrain import Drivetrain, DriveSignal
from wpimath.kinematics import ChassisSpeeds
import time
# from components.shooter import Shooter, Shooter_States

alliance = DriverStation.getAlliance()

class simple_auto(AutonomousStateMachine):
    MODE_NAME= "simple_auto"
    DEFAULT = False
    drivetrain: Drivetrain

    @timed_state(duration=10, next_state='move_after_shot', first=True)
    def shoot_preload(self):
        self.drivetrain.set_signal(DriveSignal(
            ChassisSpeeds(0, 0, 0),
            False
        ))
    
    @timed_state(duration=2.5, next_state='stopp')
    def move_after_shot(self):
        self.drivetrain.set_signal(DriveSignal(
            ChassisSpeeds(1.8, 0, 0),
            False
        ))

    @state()
    def stopp(self):
        self.drivetrain.set_signal(DriveSignal(
            ChassisSpeeds(0, 0, 0),
            False
        ))