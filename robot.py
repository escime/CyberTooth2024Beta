from commands2 import Command, CommandScheduler, TimedCommandRobot
from robotcontainer import RobotContainer
from wpilib import run, RobotBase
from phoenix6 import SignalLogger, utils
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose2d, Translation2d, Rotation2d


class Robot(TimedCommandRobot):
    """This class allows the programmer to control what runs in each individual robot operation mode."""
    m_autonomous_command: Command  # Definition for autonomous command groups used in autonomousInit
    m_robotcontainer: RobotContainer  # Type-check for robotcontainer class
    odo_ll_table = NetworkTableInstance.getDefault().getTable("limelight")
    CommandScheduler.getInstance().setPeriod(0.04)

    def robotInit(self) -> None:
        """Initialize the robot through the RobotContainer object and prep the default autonomous command (None)"""
        self.m_robotcontainer = RobotContainer()
        self.m_autonomous_command = None

    def robotPeriodic(self) -> None:
        """Set the constant robot periodic state (in command based, that's just run the scheduler loop)"""
        CommandScheduler.getInstance().run()
        self.odo_ll_table.putNumberArray("robot_orientation_set",
                                         [self.m_robotcontainer.drivetrain.get_pose().rotation().degrees(), 0, 0, 0, 0, 0])
        odo_ll_botpose = self.odo_ll_table.getEntry("botpose_orb_wpiblue").getDoubleArray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        if (odo_ll_botpose[9] < 5 and odo_ll_botpose[7] >= 1 and
                0 < odo_ll_botpose[0] < 16.7 and 0 < odo_ll_botpose[1] < 6):
            self.m_robotcontainer.drivetrain.add_vision_measurement(Pose2d(Translation2d(odo_ll_botpose[0], odo_ll_botpose[1]), Rotation2d.fromDegrees((odo_ll_botpose[5] + 360) % 360)), utils.get_current_time_seconds() - (odo_ll_botpose[6] / 1000.0), (0.2, 0.2, 999999999))

    def disabledInit(self) -> None:
        """Nothing is written here yet. Probably will not modify unless something is required for end-of-match."""

    def disabledPeriodic(self) -> None:
        """This isn't the most useful state to call anything in because you can set commands to run in disabled.
        So it's not really anything at all right now."""

    def autonomousInit(self) -> None:
        """Run the auto scheduler if the command was actually input. For the most part, this is a safety call."""
        self.m_autonomous_command = self.m_robotcontainer.get_autonomous_command()

        if self.m_autonomous_command is not None:
            self.m_autonomous_command.schedule()

    def autonomousPeriodic(self) -> None:
        """Empty for now. Handled by command scheduler."""

    def teleopInit(self) -> None:
        """Shuts off the auto command if one is being run. Could be altered to allow the command to proceed into
        teleop mode."""
        if self.m_autonomous_command:
            self.m_autonomous_command.cancel()
        self.m_robotcontainer.leds.set_state("default")

    def teleopPeriodic(self) -> None:
        """Nothing relevant here yet, everything's covered by the master scheduler."""

    def testInit(self) -> None:
        """Reset the scheduler automatically when entering test mode."""
        CommandScheduler.getInstance().cancelAll()
        self.m_robotcontainer.enable_test_bindings(True)
        if RobotBase.isReal():
            SignalLogger.set_path("/media/sda1/")
            SignalLogger.start()

    def testExit(self) -> None:
        self.m_robotcontainer.enable_test_bindings(False)
        SignalLogger.stop()

    def simulationPeriodic(self) -> None:
        """Empty for now as well."""


if __name__ == "__main__":
    run(Robot)
