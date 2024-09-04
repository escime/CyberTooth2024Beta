from commands2 import Command

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.armsubsystem import ArmSubsystem
from phoenix6 import swerve
from wpimath.controller import PIDController
from generated.tuner_constants import TunerConstants


class DriveToGamePiece(Command):
    def __init__(self, drive: CommandSwerveDrivetrain, arm: ArmSubsystem):
        super().__init__()
        self.drive = drive
        self.arm = arm

        self.drive_request = (swerve.requests.RobotCentric()
                              .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
                              .with_velocity_x(TunerConstants.speed_at_12_volts * 0.1)
                              .with_velocity_y(0))

        self.turn_controller = PIDController(0.0001, 0, 0, 0.02)

    def initialize(self):
        self.arm.set_state("intake")

    def execute(self):
        rotate_output = self.turn_controller.calculate(self.drive.tx, 0)
        self.drive.apply_request(lambda: self.drive_request.with_rotational_rate(rotate_output)).schedule()

    def isFinished(self) -> bool:
        if self.arm.get_sensor_on():
            return True
        else:
            return False

    def end(self, interrupted: bool):
        self.arm.set_state("stow")
        self.drive.apply_request(lambda: self.drive_request.with_rotational_rate(0)).withTimeout(0.02).schedule()
