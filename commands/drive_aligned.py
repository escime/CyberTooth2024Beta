from commands2 import Command

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d
from generated.tuner_constants import TunerConstants
from wpimath.units import rotationsToRadians, degreesToRadians, radiansToDegrees
from math import pi, sqrt, pow, cos, sin, atan2

from helpers.custom_hid import CustomHID
from wpilib import SmartDashboard


class DriveAligned(Command):
    def __init__(self, drive: CommandSwerveDrivetrain, target: [float, float], joystick: CustomHID):
        super().__init__()
        self.drive = drive
        self.joystick = joystick
        self.target = target

        self.forward_request = (swerve.requests.RobotCentric()
                                .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
                                .with_velocity_y(0))

        self.rotate_controller = PIDController(0.5, 0, 0, 0.02)

    def execute(self):
        x_move = self.joystick.get_axis("LY", 0.1) * -1

        current_pose = self.drive.get_pose()
        a = current_pose.x - self.target[0]
        b = current_pose.y - self.target[1]

        theta = radiansToDegrees(atan2(b, a)) + 180

        rotate_output = self.rotate_controller.calculate(current_pose.rotation().degrees(), theta)

        self.drive.apply_request(lambda: (self.forward_request.with_velocity_x(x_move * TunerConstants.speed_at_12_volts)
                                          .with_rotational_rate(rotate_output))).schedule()

    def end(self, interrupted: bool):
        self.drive.apply_request(lambda: self.forward_request).withTimeout(0.02).schedule()
