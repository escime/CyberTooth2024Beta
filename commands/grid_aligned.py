from commands2 import Command

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.utilsubsystem import UtilSubsystem
from subsystems.armsubsystem import ArmSubsystem
from phoenix6 import swerve
from wpimath.controller import PIDController
from generated.tuner_constants import TunerConstants
from wpimath.units import degreesToRadians, radiansToDegrees
from math import sqrt, pow, cos, sin, atan2

from helpers.custom_hid import CustomHID


class GridAligned(Command):
    def __init__(self, drive: CommandSwerveDrivetrain, util: UtilSubsystem, arm: ArmSubsystem, x: float,
                 approach_angle: float, flipped: bool, joystick: CustomHID):
        super().__init__()
        self.drive = drive
        self.joystick = joystick
        self.util = util
        self.arm = arm
        self.approach_angle = approach_angle
        self.flipped = flipped
        self.x = x

        self.forward_request = (swerve.requests.RobotCentric()
                                .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
                                .with_velocity_y(0))

        self.rotate_controller = PIDController(0.1, 0, 0, 0.02)
        self.rotate_controller.enableContinuousInput(-180, 180)
        self.y_controller = PIDController(0.4, 0, 0, 0.02)
        self.target = [0, 0]

    def initialize(self):
        self.target = [self.x, self.util.get_grid_setpoint()[1]]
        self.arm.set_state(self.util.get_grid_setpoint()[0])

    def execute(self):
        x_move = self.joystick.get_axis("LY", 0.1) * -1

        current_pose = self.drive.get_pose()
        a = current_pose.x - self.target[0]
        b = current_pose.y - self.target[1]

        theta = radiansToDegrees(atan2(b, a))
        if self.flipped:
            theta += 180
            x_move = x_move * -1

        rotate_output = self.rotate_controller.calculate(current_pose.rotation().degrees(), theta + 180)
        y_output = self.y_controller.calculate(self.get_vector_to_line(current_pose, self.approach_angle), 0)

        if -0.05 < y_output < 0.05:
            y_output = 0

        self.drive.apply_request(lambda: (self.forward_request
                                          .with_velocity_x(x_move * TunerConstants.speed_at_12_volts)
                                          .with_rotational_rate(rotate_output)
                                          .with_velocity_y(y_output * TunerConstants.speed_at_12_volts))).schedule()

    def end(self, interrupted: bool):
        self.drive.apply_request(lambda: self.forward_request).withTimeout(0.02).schedule()
        self.arm.set_state("stow")

    def get_closest_target_coordinates(self, current_pose, alpha) -> [float, float]:
        """When given x, output y."""
        y2 = self.target[1]
        x2 = self.target[0]
        c = 2

        y1 = y2 - c * cos(degreesToRadians(alpha))
        x1 = x2 - c * sin(degreesToRadians(alpha))
        m = (y2 - y1) / (x2 - x1)
        b = y2 - (m * x2)

        xmin = (current_pose.x + (m * current_pose.y) - (b * m)) / (pow(m, 2) + 1)
        ymin = ((-1 * m * (-1 * current_pose.x - (m * current_pose.y))) + b) / (pow(m, 2) + 1)

        return xmin, ymin

    def get_point_on_line(self, x: float, alpha) -> float:
        y2 = self.target[1]
        x2 = self.target[0]
        c = 2

        y1 = y2 - c * cos(degreesToRadians(alpha))
        x1 = x2 - c * sin(degreesToRadians(alpha))
        m = (y2 - y1) / (x2 - x1)
        b = y2 - (m * x2)

        return (m * x) + b

    def get_distance_to_line(self, current_pose, alpha) -> float:
        y2 = self.target[1]
        x2 = self.target[0]
        c = 2

        y1 = y2 - c * cos(degreesToRadians(alpha))
        x1 = x2 - c * sin(degreesToRadians(alpha))
        m = (y2 - y1) / (x2 - x1)
        b = y2 - (m * x2)

        return abs((-1 * m * current_pose.x) + current_pose.y - b) / sqrt(pow(m, 2) + pow(1, 2))

    def get_vector_to_line(self, current_pose, alpha):
        xmin, ymin = self.get_closest_target_coordinates(current_pose, alpha)
        if not self.flipped:
            if current_pose.x > xmin:
                return -1 * self.get_distance_to_line(current_pose, alpha)
            elif current_pose.x < xmin:
                return self.get_distance_to_line(current_pose, alpha)
            else:
                return 0
        else:
            if current_pose.x < xmin:
                return -1 * self.get_distance_to_line(current_pose, alpha)
            elif current_pose.x > xmin:
                return self.get_distance_to_line(current_pose, alpha)
            else:
                return 0
