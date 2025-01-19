from magicbot import AutonomousStateMachine, state, tunable
import wpilib
from wpimath.geometry import Pose2d
import choreo


from components.chassis.drivetrain import Drivetrain
import utilities.ozone_utility_functions as utils



class Autonomous(AutonomousStateMachine):  
    MODE_NAME = "Swerve Trajectory Following"
    DEFAULT = True
    
    drivetrain: Drivetrain  # Inject the Drive subsystem

    def __init__(self) -> None:
        super().__init__()
        self.timer = wpilib.Timer()
        
        try:
            self.trajectory = choreo.load_swerve_trajectory('/home/lvuser/py/Autonomous_Paths/Rotate')
            self.starting_pose = self.trajectory.get_initial_pose(False)
            self.final_pose = self.trajectory.get_final_pose(False)
        except ValueError:
            self.trajectory = choreo.SwerveTrajectory("", [], [], [])
            self.starting_pose = None   

        
    def on_enable(self) -> None:
        # self.starting_pose = self.get_initial_pose(self.starting_pose)
        super().on_enable()
        

    @state(first=True)
    def initialize(self):
        """Initial state of the autonomous mode."""
        if self.starting_pose is not None:
            self.drivetrain.reset_odometry(self.starting_pose)
            for i in range(20):
                print(self.starting_pose, self.final_pose)
                  
        self.timer.restart()
        self.timer.start()
        self.next_state("follow_trajectory")  # Move to the next state

    @state
    def follow_trajectory(self):
        """Follow the loaded trajectory."""
        if self.trajectory:
            sample = self.trajectory.sample_at(self.timer.get(), False)
            if sample:
                self.drivetrain.follow_trajectory(sample)
                
    # def get_initial_pose(self, p: Pose2d):
    #     pose = p
    #     if p is None:
    #         return None
        
    #     if self.is_red():
    #         utils.flip_pose_2d(pose)
        
        # return pose
               
    @staticmethod
    def is_red():
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed