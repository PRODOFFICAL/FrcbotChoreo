from wpilib.shuffleboard import Shuffleboard, BuiltInWidgets
import wpilib

from components.chassis.drivetrain import Drivetrain
from components.chassis.swervemodule import SwerveModule

import utilities.constants as constants

WINDOW_WIDTH = 10

class SbTab():

    instance = None

    def __init__(self) -> None:
        SbTab.instance = self
        self.components_initialized = False

        self.main_tab = Shuffleboard.getTab("Main")
        self.debug_tab = Shuffleboard.getTab("Debug")
        self.pose_tab = Shuffleboard.getTab("Pose_Testing")
        
        self.sport_mode = self.main_tab.add("Sport Mode", False).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(WINDOW_WIDTH - 2, 3).getEntry()

        self.leds_enabled = self.debug_tab.add("LEDS enable", True).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(WINDOW_WIDTH - 3, 3).getEntry()
        self.climber_enabled = self.debug_tab.add("Climber enable", True).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(WINDOW_WIDTH - 4, 3).getEntry()
        self.intake_enabled = self.debug_tab.add("Intake enable", True).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(WINDOW_WIDTH - 5, 3).getEntry()
        self.shooter_enabled = self.debug_tab.add("Shooter enable", True).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(WINDOW_WIDTH - 6, 3).getEntry()

        self.front_left_deg = self.debug_tab.add("FL Degrees", 0).getEntry()
        self.front_left_deg_corrected = self.debug_tab.add("FL Degrees Corrected", 0).withPosition(1,0).getEntry()
        self.front_right_deg = self.debug_tab.add("FR Degrees", 0).withPosition(2,0).getEntry()
        self.front_right_deg_corrected= self.debug_tab.add("FR Degrees Corrected", 0).withPosition(3,0).getEntry()
        self.back_left_deg = self.debug_tab.add("BL Degrees", 0).withPosition(4,0).getEntry()
        self.back_left_deg_corrected = self.debug_tab.add("BL Degrees Corrected", 0).withPosition(5,0).getEntry()
        self.back_right_deg = self.debug_tab.add("FL Degrees", 0).withPosition(6,0).getEntry()
        self.back_right_deg_corrected = self.debug_tab.add("FL Degrees Corrected", 0).withPosition(7,0).getEntry()

        self.test_estimation_x = self.pose_tab.add("Pose_Estimator_X", 0).getEntry()
        self.test_estimation_y = self.pose_tab.add("Pose_Estimator_Y", 0).getEntry()
        self.test_estimation_rot = self.pose_tab.add("Pose_Estimator_ROT", 0).getEntry()

        self.gyro_angle = self.main_tab.add("Gyro Angle", 0).withWidget(BuiltInWidgets.kGyro).withPosition(1, 0).getEntry()
        self.field_relative = self.main_tab.add("Field Relative", Drivetrain.instance.field_relative if not Drivetrain.instance is None else False).withPosition(3, 0).getEntry()

        #Shooter angle displays
        self.shooter_at_target_position = self.debug_tab.add("Shooter at target angle?", False).withPosition(0, 1).getEntry()
        self.shooter_target_angle = self.debug_tab.add("Shooter Target Angle", 0).withPosition(1, 1).getEntry()
        self.shooter_angle = self.debug_tab.add("Shooter angle", 0).withPosition(2, 1).getEntry()

        #Shooter speed displays
        self.shooter_at_target_speed = self.debug_tab.add("Shooter at target speed?", False).withPosition(4, 1).getEntry()
        self.shooter_target_speed = self.debug_tab.add("Shooter target speed", 0).withPosition(5, 1).getEntry()
        self.shooter_speed = self.debug_tab.add("Shooter speed", 0).withPosition(6, 1).getEntry()

        #Intake
        self.intake_has_note = self.debug_tab.add("Has note", False).withPosition(0, 2).getEntry()
        self.intake_at_target_posion = self.debug_tab.add("Intake at target position", False).withPosition(1, 2).getEntry()
        self.intake_pivot_target = self.debug_tab.add("Pivot target", 0).withPosition(2, 2).getEntry()
        self.intake_pivot_position = self.debug_tab.add("Pivot position", 0).withPosition(3, 2).getEntry()

        # work around because widgets are not applied to last object created for some reason
        self.timer = self.main_tab.add("Drive Time", 0).getEntry()
    
    def is_sport_mode(self) -> bool:
            return self.sport_mode.getBoolean(False)

    def is_climber_enabled(self) -> bool:
            return self.climber_enabled.getBoolean(True)

    def is_intake_enabled(self) -> bool:
            return self.intake_enabled.getBoolean(False)

    def is_shooter_enabled(self) -> bool:
            return self.shooter_enabled.getBoolean(False)

    def are_leds_enabled(self) -> bool:
            return self.leds_enabled.getBoolean(False)
    

    def update(self) -> None: #Use for anything than needs to be updated
        self.timer.setDouble(wpilib.DriverStation.getMatchTime())
          
        if not Drivetrain.instance is None:
                FL_angle = Drivetrain.instance.front_left.get_encoder_angle_deg()
                FR_angle = Drivetrain.instance.front_right.get_encoder_angle_deg()
                BL_angle = Drivetrain.instance.back_left.get_encoder_angle_deg()
                BR_angle = Drivetrain.instance.back_right.get_encoder_angle_deg()

                self.gyro_angle.setDouble(Drivetrain.instance.get_yaw_value_degrees())
                self.field_relative.setBoolean(Drivetrain.instance.field_relative)

                self.front_left_deg.setDouble(FL_angle)
                self.front_left_deg_corrected.setDouble(FL_angle - constants.FL_OFFSET)

                self.front_right_deg.setDouble(FR_angle)
                self.front_right_deg_corrected.setDouble(FR_angle - constants.FR_OFFSET)

                self.back_left_deg.setDouble(BL_angle)
                self.back_left_deg_corrected.setDouble(BL_angle - constants.BL_OFFSET)

                self.back_right_deg.setDouble(BR_angle)
                self.back_right_deg_corrected.setDouble(BR_angle - constants.BR_OFFSET)

                self.test_estimation_x.setDouble(Drivetrain.instance.pose_estimator.getEstimatedPosition().translation().x)
                self.test_estimation_y.setDouble(Drivetrain.instance.pose_estimator.getEstimatedPosition().translation().y)
                self.test_estimation_rot.setDouble(Drivetrain.instance.pose_estimator.getEstimatedPosition().rotation().degrees() % 360)
               