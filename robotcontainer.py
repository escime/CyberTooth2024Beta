from commands2.cmd import run, runOnce, runEnd
import wpilib.simulation
from commands2 import Command, button, SequentialCommandGroup, ParallelCommandGroup, ParallelRaceGroup, sysid, \
    InterruptionBehavior, ParallelDeadlineGroup, WaitCommand

from constants import OIConstants
from subsystems.ledsubsystem import LEDs
from subsystems.armsubsystem import ArmSubsystem
from subsystems.utilsubsystem import UtilSubsystem
from wpilib import SmartDashboard, SendableChooser, DriverStation, DataLogManager, Timer, Alert
from pathplannerlib.auto import NamedCommands, PathPlannerAuto
from wpinet import PortForwarder

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6 import swerve, SignalLogger
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

from math import pi

from commands.baseline import Baseline
from commands.check_drivetrain import CheckDrivetrain
from commands.alignment_leds import AlignmentLEDs
from commands.drive_to_gamepiece import DriveToGamePiece
from commands.profiled_target import ProfiledTarget
from commands.auto_alignment_auto_select import AutoAlignmentAutoSelect


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # Start master timer. ------------------------------------------------------------------------------------------
        self.timer = Timer()
        self.timer.start()

        # Configure button to enable robot logging.
        self.logging_button = SmartDashboard.putBoolean("Logging Enabled?", False)

        # Disable automatic ctre logging
        SignalLogger.enable_auto_logging(False)

        # Configure system logging. ------------------------------------------------------------------------------------
        self.alert_logging_enabled = Alert("Robot Logging is Enabled", Alert.AlertType.kWarning)
        self.alert_limelight = Alert("Limelight ports forwarded", Alert.AlertType.kWarning)
        if wpilib.RobotBase.isReal():
            if SmartDashboard.getBoolean("Logging Enabled?", False) is True:
                DataLogManager.start()
                DriverStation.startDataLog(DataLogManager.getLog(), True)
                SignalLogger.start()
                self.alert_logging_enabled.set(True)
            else:
                SignalLogger.stop()
            for port in range(5800, 5810):
                PortForwarder.getInstance().add(port, "10.39.40.11", port)
            for port in range(5800, 5810):
                PortForwarder.getInstance().add(port+10, "10.39.40.12", port)
            self.alert_limelight.set(True)

        # Startup subsystems. ------------------------------------------------------------------------------------------
        self.leds = LEDs(self.timer)
        self.arm = ArmSubsystem()
        self.util = UtilSubsystem()

        # Setup driver & operator controllers. -------------------------------------------------------------------------
        # self.driver_controller = CustomHID(OIConstants.kDriverControllerPort, "xbox")
        # self.operator_controller = CustomHID(OIConstants.kOperatorControllerPort, "xbox")
        self.driver_controller = button.CommandXboxController(OIConstants.kDriverControllerPort)
        self.operator_controller = button.CommandXboxController(OIConstants.kOperatorControllerPort)
        DriverStation.silenceJoystickConnectionWarning(True)
        self.test_bindings = False

        # Configure drivetrain settings. -------------------------------------------------------------------------------
        self._max_speed = TunerConstants.speed_at_12_volts  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(0.75)  # 3/4 of a rotation per second max angular velocity

        self._logger = Telemetry(self._max_speed)

        self.drivetrain = TunerConstants.create_drivetrain()

        self._drive = (
            swerve.requests.FieldCentric()  # I want field-centric
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(self._max_angular_rate * 0.1)  # Add a 10% deadband
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_desaturate_wheel_speeds(True)
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._hold_heading = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(self._max_speed * 0.1)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_desaturate_wheel_speeds(True)
        )
        self._hold_heading.heading_controller.setPID(5, 0, 0)
        self._hold_heading.heading_controller.enableContinuousInput(0, -2 * pi)
        self._hold_heading.heading_controller.setTolerance(0.1)

        # Register commands for PathPlanner. ---------------------------------------------------------------------------
        self.registerCommands()

        # Set up new autonomous selection structure
        self.m_auto_start_location = SendableChooser()
        self.m_auto_start_location.setDefaultOption("A", "A")
        self.m_auto_start_location.addOption("B", "B")
        self.m_auto_start_location.addOption("C", "C")
        self.m_auto_num_gp = SendableChooser()
        self.m_auto_num_gp.setDefaultOption("1", "1")
        self.m_auto_num_gp.addOption("2", "2")
        self.m_auto_num_gp.addOption("3", "3")
        self.m_auto_num_gp.addOption("4", "4")
        self.m_auto_num_gp.addOption("5", "5")
        SmartDashboard.putData("Auto Start Selector", self.m_auto_start_location)
        SmartDashboard.putData("Auto GP Num Selector", self.m_auto_num_gp)

        SmartDashboard.putBoolean("Misalignment Indicator Active?", False)
        SmartDashboard.putNumber("Misalignment Angle", 0)

        # Setup for all event-trigger commands. ------------------------------------------------------------------------
        # self.configureTriggersDefault()
        self.configure_test_bindings()
        self.configure_triggers_rewrite()

        # Setup autonomous selector on the dashboard. ------------------------------------------------------------------
        self.m_chooser = SendableChooser()
        self.auto_names = ["Test", "TestChoreo", "Baseline", "CheckDrivetrain", "FourNote", "BuildPlay"]
        self.m_chooser.setDefaultOption("DoNothing", "DoNothing")
        for x in self.auto_names:
            self.m_chooser.addOption(x, x)
        SmartDashboard.putData("Auto Select", self.m_chooser)

    def configure_triggers_rewrite(self) -> None:
        self.drivetrain.setDefaultCommand(  # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self.driver_controller.getLeftY() * self._max_speed)
                    .with_velocity_y(-self.driver_controller.getLeftX() * self._max_speed)
                    .with_rotational_rate(-self.driver_controller.getRightX() * self._max_angular_rate)
                )
            )
        )

        # Closed loop turning on command implementation.
        self.driver_controller.leftBumper().and_(lambda: not self.test_bindings).onTrue(
            SequentialCommandGroup(
                runOnce(lambda: self.drivetrain.reset_clt(), self.drivetrain),
                self.drivetrain.apply_request(
                    lambda: (
                        self.drivetrain.drive_clt(
                            self.driver_controller.getLeftY() * self._max_speed * -1,
                            self.driver_controller.getLeftX() * self._max_speed * -1,
                            self.driver_controller.getRightX() * -1
                        )
                    )
                )
            )
        )

        # Reset pose.
        self.driver_controller.y().and_(lambda: not self.test_bindings).onTrue(
            runOnce(lambda: self.drivetrain.reset_odometry(), self.drivetrain))

        # Auto Alignment
        self.driver_controller.b().and_(lambda: not self.test_bindings).whileTrue(
            ParallelCommandGroup(
                AlignmentLEDs(self.leds, self.drivetrain),
                ProfiledTarget(self.drivetrain, self.arm, [16.5, 5.53])
            ))

        # Arm manual controls.
        self.driver_controller.povUp().and_(lambda: not self.test_bindings).whileTrue(
            run(lambda: self.arm.set_voltage_direct(1), self.arm)).onFalse(
            run(lambda: self.arm.set_voltage_direct(0), self.arm))
        self.driver_controller.povDown().and_(lambda: not self.test_bindings).whileTrue(
            run(lambda: self.arm.set_voltage_direct(-1), self.arm)).onFalse(
            run(lambda: self.arm.set_voltage_direct(0), self.arm))

        # Auto selecting auto alignment.
        self.driver_controller.rightTrigger().and_(lambda: not self.test_bindings).whileTrue(
            AutoAlignmentAutoSelect(self.drivetrain, self.util, self.arm, True, self.driver_controller))

        # Shoot over the non-intake side
        self.driver_controller.rightBumper().and_(lambda: not self.test_bindings).onTrue(
            runOnce(lambda: self.arm.set_state("reverse_shoot"), self.arm)
        ).onFalse(
            runOnce(lambda: self.arm.set_state("stow"), self.arm)
        )

        # Intake
        self.driver_controller.leftTrigger().and_(lambda: not self.test_bindings).onTrue(
            runOnce(lambda: self.arm.set_state("intake"), self.arm)
        ).onFalse(
            runOnce(lambda: self.arm.set_state("stow"), self.arm)
        )

        # Cycle through scoring set points.
        self.driver_controller.povLeft().and_(lambda: not self.test_bindings).onTrue(
            runOnce(lambda: self.util.cycle_scoring_setpoints(1), self.util).ignoringDisable(True)
        )
        self.driver_controller.povRight().and_(lambda: not self.test_bindings).onTrue(
            runOnce(lambda: self.util.cycle_scoring_setpoints(-1), self.util).ignoringDisable(True)
        )

        # Cube acquired light
        button.Trigger(lambda: self.arm.get_sensor_on() and DriverStation.isTeleop()).onTrue(
            SequentialCommandGroup(
                runOnce(lambda: self.leds.set_flash_color_rate(10), self.leds),
                runOnce(lambda: self.leds.set_flash_color_color([0, 255, 0]), self.leds),
                runOnce(lambda: self.leds.set_state("flash_color"), self.leds),
                WaitCommand(2),
                runOnce(lambda: self.leds.set_state("gp_held"), self.leds)
            ).ignoringDisable(True)
        ).onFalse(
            SequentialCommandGroup(
                runOnce(lambda: self.leds.set_flash_color_rate(10), self.leds),
                runOnce(lambda: self.leds.set_flash_color_color([255, 0, 0]), self.leds),
                runOnce(lambda: self.leds.set_state("flash_color"), self.leds),
                WaitCommand(0.5),
                runOnce(lambda: self.leds.set_state("default"), self.leds)
            ).ignoringDisable(True)
        )

        # Servo Testing
        self.driver_controller.x().onTrue(
            runOnce(lambda: self.arm.set_servo(1), self.arm)
        ).onFalse(
            runOnce(lambda: self.arm.set_servo(0), self.arm)
        )

        # Configuration for telemetry.
        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def configureTriggersDefault(self) -> None:
        print("Keeping this around for just a minute while i decide if there's a better way to do this.")
        # Activate autonomous misalignment lights.
        # button.Trigger(lambda: SmartDashboard.getBoolean("Misalignment Indicator Active?", False)).whileTrue(
        #     AutoAlignmentLEDs(self.drivetrain, self.leds, self.m_auto_start_location)
        #     .ignoringDisable(True)
        # )

        # button.Trigger(lambda: SmartDashboard.getBoolean("Logging Enabled?", False)).onTrue(
        #     SequentialCommandGroup(
        #         runOnce(lambda: DataLogManager.start()),
        #         runOnce(lambda: DriverStation.startDataLog(DataLogManager.getLog(), True)),
        #         runOnce(lambda: SignalLogger.start()),
        #         runOnce(lambda: self.alert_logging_enabled.set(True))
        #     )
        # ).onFalse(
        #     SequentialCommandGroup(
        #         runOnce(lambda: DataLogManager.stop()),
        #         runOnce(lambda: SignalLogger.stop()),
        #         runOnce(lambda: self.alert_logging_enabled.set(False))
        #     )
        # )
    def get_autonomous_command(self) -> Command:
        """Use this to pass the autonomous command to the main Robot class.
        Returns the command to run in autonomous
        """
        if self.m_chooser.getSelected() == "DoNothing":
            return None
        elif self.m_chooser.getSelected() == "BuildPlay":
            try:
                selected_auto = PathPlannerAuto(self.m_auto_start_location.getSelected() + "_Score" +
                                                self.m_auto_num_gp.getSelected())
            except FileNotFoundError:
                selected_auto = None
            return selected_auto
        else:
            selected_auto = None
            for y in self.auto_names:
                if self.m_chooser.getSelected() == y:
                    try:
                        selected_auto = PathPlannerAuto(y)
                    except FileNotFoundError:
                        selected_auto = None
            return selected_auto

    def configure_test_bindings(self) -> None:
        self.configure_sys_id()

        # Point all modules in a direction
        self.driver_controller.start().and_(lambda: self.test_bindings).whileTrue(self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-1 * self.driver_controller.getLeftY()
                               -1 * self.driver_controller.getLeftX()))))

    def configure_sys_id(self) -> None:
        (self.driver_controller.y().and_(lambda: self.test_bindings).and_(self.driver_controller.rightTrigger())
         .whileTrue(self.drivetrain.sys_id_translation_quasistatic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.b().and_(lambda: self.test_bindings).and_(self.driver_controller.rightTrigger())
         .whileTrue(self.drivetrain.sys_id_translation_quasistatic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.a().and_(lambda: self.test_bindings).and_(self.driver_controller.rightTrigger())
         .whileTrue(self.drivetrain.sys_id_translation_dynamic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.x().and_(lambda: self.test_bindings).and_(self.driver_controller.rightTrigger())
         .whileTrue(self.drivetrain.sys_id_translation_dynamic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.y().and_(lambda: self.test_bindings).and_(self.driver_controller.rightBumper())
         .whileTrue(self.drivetrain.sys_id_rotation_quasistatic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.b().and_(lambda: self.test_bindings).and_(self.driver_controller.rightBumper())
         .whileTrue(self.drivetrain.sys_id_rotation_quasistatic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.a().and_(lambda: self.test_bindings).and_(self.driver_controller.rightBumper())
         .whileTrue(self.drivetrain.sys_id_rotation_dynamic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.x().and_(lambda: self.test_bindings).and_(self.driver_controller.rightBumper())
         .whileTrue(self.drivetrain.sys_id_rotation_dynamic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.y().and_(lambda: self.test_bindings).and_(self.driver_controller.leftBumper())
         .whileTrue(self.drivetrain.sys_id_steer_quasistatic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.b().and_(lambda: self.test_bindings).and_(self.driver_controller.leftBumper())
         .whileTrue(self.drivetrain.sys_id_steer_quasistatic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.a().and_(lambda: self.test_bindings).and_(self.driver_controller.leftBumper())
         .whileTrue(self.drivetrain.sys_id_steer_dynamic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.x().and_(lambda: self.test_bindings).and_(self.driver_controller.leftBumper())
         .whileTrue(self.drivetrain.sys_id_steer_dynamic(sysid.SysIdRoutine.Direction.kReverse)))

    def enable_test_bindings(self, enabled: bool) -> None:
        self.test_bindings = enabled

    def registerCommands(self):
        NamedCommands.registerCommand("rainbow_leds", runOnce(lambda: self.leds.set_state("rainbow"),
                                                              self.leds))
        NamedCommands.registerCommand("flash_green",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.leds.set_flash_color_color([255, 0, 0]),
                                                  self.leds),
                                          runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("flash_red",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.leds.set_flash_color_color([0, 255, 0]),
                                                  self.leds),
                                          runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("flash_blue",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.leds.set_flash_color_color([0, 0, 255]),
                                                  self.leds),
                                          runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("flash_purple",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.leds.set_flash_color_color([50, 149, 168]),
                                                  self.leds),
                                          runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("flash_yellow",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.leds.set_flash_color_color([255, 255, 0]),
                                                  self.leds),
                                          runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("default_leds", runOnce(lambda: self.leds.set_state("default"),
                                                              self.leds))
        NamedCommands.registerCommand("baseline", Baseline(self.drivetrain, self.timer))
        NamedCommands.registerCommand("check_drivetrain", CheckDrivetrain(self.drivetrain, self.timer))
        NamedCommands.registerCommand("override_heading_goal",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.drivetrain.set_lookahead(True)),
                                          runOnce(lambda: self.drivetrain.set_pathplanner_rotation_override("goal"))
                                        )
                                      )
        NamedCommands.registerCommand("override_heading_gp",
                                      runOnce(lambda: self.drivetrain.set_pathplanner_rotation_override("gp")))
        NamedCommands.registerCommand("disable_override_heading",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.drivetrain.set_lookahead(False)),
                                          runOnce(lambda: self.drivetrain.set_pathplanner_rotation_override("none"))
                                      ))
        NamedCommands.registerCommand("intake", runOnce(lambda: self.arm.set_state("intake"),
                                                        self.arm))
        NamedCommands.registerCommand("stow", runOnce(lambda: self.arm.set_state("stow"), self.arm))
        NamedCommands.registerCommand("reverse_shoot", runOnce(lambda: self.arm.set_state("reverse_shoot"),
                                                               self.arm))
