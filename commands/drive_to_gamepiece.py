from commands2 import Command

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.armsubsystem import ArmSubsystem
from phoenix6 import swerve
from wpimath.controller import PIDController
from generated.tuner_constants import TunerConstants
from wpimath.units import rotationsToRadians

from wpilib import SmartDashboard


class DriveToGamePiece(Command):
    def __init__(self, drive: CommandSwerveDrivetrain, arm: ArmSubsystem):
        super().__init__()
        self.drive = drive
        self.arm = arm
        self.gp_acquired = [False] * 50

        self.drive_request = (swerve.requests.RobotCentric()
                              .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
                              .with_velocity_x(TunerConstants.speed_at_12_volts * 0.3)
                              .with_velocity_y(0))

        self.turn_controller = PIDController(0.05, 0, 0, 0.02)
        SmartDashboard.putData("Game Piece Tracking Controller", self.turn_controller)

    def initialize(self):
        self.arm.set_state("intake")

    def execute(self):
        rotate_output = self.turn_controller.calculate(self.drive.tx, 0)
        if rotate_output > rotationsToRadians(0.75):
            rotate_output = rotationsToRadians(0.75)
        elif rotate_output < -1 * rotationsToRadians(0.75):
            rotate_output = -1 * rotationsToRadians(0.75)
        if self.arm.get_at_target():
            self.drive.apply_request(lambda: self.drive_request.with_rotational_rate(rotate_output)).schedule()

        if self.arm.get_sensor_on():
            self.gp_acquired[0] = True
        else:
            self.gp_acquired[0] = False
        self.gp_acquired = self.gp_acquired[1:] + self.gp_acquired[:1]

    def isFinished(self) -> bool:
        return all(self.gp_acquired)

    def end(self, interrupted: bool):
        self.arm.set_state("stow")
        self.drive.apply_request(lambda: self.drive_request.with_rotational_rate(0)).withTimeout(0.02).schedule()
