from commands2 import Command

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.utilsubsystem import UtilSubsystem
from subsystems.armsubsystem import ArmSubsystem
from phoenix6 import swerve
from wpimath.controller import PIDController
from generated.tuner_constants import TunerConstants
from wpimath.units import degreesToRadians, radiansToDegrees
from math import sqrt, pow, cos, sin, atan2
from wpilib import DriverStation

from helpers.custom_hid import CustomHID


class AutoAlignment(Command):
    def __init__(self, drive: CommandSwerveDrivetrain, util: UtilSubsystem, arm: ArmSubsystem, flipped: bool,
                 joystick: CustomHID):
        super().__init__()
        self.drive = drive
        self.joystick = joystick
        self.util = util
        self.arm = arm
        self.flipped = flipped

        self.forward_request = (swerve.requests.RobotCentric()
                                .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
                                .with_velocity_y(0))

        self.rotate_controller = PIDController(0.1, 0, 0, 0.02)
        self.rotate_controller.enableContinuousInput(-180, 180)
        self.y_controller = PIDController(0.5, 0, 0, 0.02)
        self.target = [0, 0]

        # self.addRequirements(self.drive)
        self.addRequirements(self.arm)

    def initialize(self):
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.target = [self.util.scoring_locations_red[self.util.scoring_location][0],
                           self.util.scoring_locations_red[self.util.scoring_location][1]]
        else:
            self.target = [self.util.scoring_locations_blue[self.util.scoring_location][0],
                           self.util.scoring_locations_blue[self.util.scoring_location][1]]
        self.arm.set_state(self.util.scoring_setpoints[self.util.scoring_setpoint])

    def execute(self):
        x_move = self.joystick.get_axis("LY", 0.1) * -1

        current_pose = self.drive.get_pose()
        a = self.target[0] - current_pose.x
        b = self.target[1] - current_pose.y

        theta = radiansToDegrees(atan2(b, a))

        if theta < 0:
            theta = 360 + theta

        if self.flipped:
            theta += 180
            x_move = x_move * -1

        current_heading = current_pose.rotation().degrees()
        if current_heading < 0:
            current_heading = 360 + current_heading

        rotate_output = self.rotate_controller.calculate(current_heading, theta)
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            y_output = self.y_controller.calculate(
                self.get_vector_to_line(current_pose, self.util.scoring_locations_red[self.util.scoring_location][2]),
                0)
        else:
            y_output = self.y_controller.calculate(
                self.get_vector_to_line(current_pose, self.util.scoring_locations_blue[self.util.scoring_location][2]),
                0)
        # print("Target Angle: ", str(self.util.scoring_locations[self.util.scoring_location][2]))

        if -0.02 < y_output < 0.02:
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
        y1 = self.target[1]
        x1 = self.target[0]
        c = 2

        # TODO Angles greater than 180 currently don't work lol
        y2 = y1 + c * sin(degreesToRadians(alpha))
        x2 = x1 + c * cos(degreesToRadians(alpha))
        m = (y2 - y1) / (x2 - x1)
        b = y2 - (m * x2)

        # print("y2: " + str(y2))
        # print("x2: " + str(x2))
        # print("current pose x: " + str(current_pose.x))
        # print("current pose y: " + str(current_pose.y))
        # print("y = " + str(m) + "x + " + str(b))

        xmin = (current_pose.x + (m * current_pose.y) - (b * m)) / (pow(m, 2) + 1)
        ymin = ((-1 * m * (-1 * current_pose.x - (m * current_pose.y))) + b) / (pow(m, 2) + 1)

        if alpha > 180:
            ymin = current_pose.y - (ymin - current_pose.y)

        # if alpha > 180.001:
        #     ymin = (self.target[1] - ymin) + self.target[1]

        # print("xmin: " + str(xmin))
        # print("ymin: " + str(ymin))

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
        y1 = self.target[1]
        x1 = self.target[0]
        c = 2

        y2 = y1 + c * sin(degreesToRadians(alpha))
        x2 = x1 + c * cos(degreesToRadians(alpha))
        m = (y2 - y1) / (x2 - x1)
        b = y2 - (m * x2)

        return abs((-1 * m * current_pose.x) + current_pose.y - b) / sqrt(pow(-1 * m, 2) + 1)

    def get_vector_to_line(self, current_pose, alpha):
        xmin, ymin = self.get_closest_target_coordinates(current_pose, alpha)

        print("current pose x: " + str(current_pose.x))
        print("current pose y: " + str(current_pose.y))
        print("xmin: " + str(xmin))
        print("ymin: " + str(ymin))
        if not self.flipped:
            if ymin > current_pose.y:
                return -1 * self.get_distance_to_line(current_pose, alpha)
            elif ymin < current_pose.y:
                return self.get_distance_to_line(current_pose, alpha)
            else:
                return 0
        else:
            if ymin < current_pose.y:
                return -1 * self.get_distance_to_line(current_pose, alpha)
            elif ymin > current_pose.y:
                return self.get_distance_to_line(current_pose, alpha)
            else:
                return 0

        # if not self.flipped:
        #     if current_pose.x > xmin:
        #         return -1 * self.get_distance_to_line(current_pose, alpha)
        #     elif current_pose.x < xmin:
        #         return self.get_distance_to_line(current_pose, alpha)
        #     else:
        #         return 0
        # else:
        #     if current_pose.x < xmin:
        #         return -1 * self.get_distance_to_line(current_pose, alpha)
        #     elif current_pose.x > xmin:
        #         return self.get_distance_to_line(current_pose, alpha)
        #     else:
        #         return 0
