from commands2.cmd import run, runOnce, runEnd
import wpilib.simulation
from commands2 import Command, button, SequentialCommandGroup, ParallelCommandGroup, ParallelRaceGroup, sysid, \
    InterruptionBehavior, ParallelDeadlineGroup, WaitCommand

from constants import OIConstants
from subsystems.ledsubsystem import LEDs
from subsystems.elevatorsubsystem import ElevatorSubsystem
from wpilib import SmartDashboard, SendableChooser, DriverStation, DataLogManager, Timer
from helpers.custom_hid import CustomHID
from pathplannerlib.auto import NamedCommands, PathPlannerAuto

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6 import swerve, utils, SignalLogger
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import rotationsToRadians

from math import pi

from commands.baseline import Baseline
from commands.check_drivetrain import CheckDrivetrain
from commands.alignment_leds import AlignmentLEDs


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # Start master timer. ------------------------------------------------------------------------------------------
        # TODO Is this necessary? Is it just for filed commands?
        self.timer = Timer()
        self.timer.start()

        # Configure system logging. ------------------------------------------------------------------------------------
        if wpilib.RobotBase.isReal():
            print("Not a simulation, logging enabled!")
            DataLogManager.start()
            DriverStation.startDataLog(DataLogManager.getLog(), True)
        else:
            print("Simulated, logging disabled.")

        # Startup subsystems. ------------------------------------------------------------------------------------------
        self.leds = LEDs(self.timer)
        self.elevator = ElevatorSubsystem()

        # Setup driver & operator controllers. -------------------------------------------------------------------------
        self.driver_controller = CustomHID(OIConstants.kDriverControllerPort, "xbox")
        self.operator_controller = CustomHID(OIConstants.kOperatorControllerPort, "xbox")
        DriverStation.silenceJoystickConnectionWarning(True)
        self.test_bindings = False

        # Configure drivetrain settings. -------------------------------------------------------------------------------
        self._max_speed = TunerConstants.speed_at_12_volts  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(0.75)  # 3/4 of a rotation per second max angular velocity

        self._logger = Telemetry(self._max_speed)

        self.drivetrain = TunerConstants.create_drivetrain()

        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(self._max_angular_rate * 0.1)  # Add a 10% deadband
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)  # I want field-centric
            .with_desaturate_wheel_speeds(True)
            # driving in open loop
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._hold_heading = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(self._max_speed * 0.1)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
            .with_desaturate_wheel_speeds(True)
        )
        self._hold_heading.heading_controller.setPID(5, 0, 0)
        self._hold_heading.heading_controller.enableContinuousInput(0, -2 * pi)
        self._hold_heading.heading_controller.setTolerance(0.1)

        # Register commands for PathPlanner. ---------------------------------------------------------------------------
        self.registerCommands()

        # Setup for all event-trigger commands. ------------------------------------------------------------------------
        self.configureTriggersDefault()
        self.configureTestBindings()

        # Setup autonomous selector on the dashboard. ------------------------------------------------------------------
        self.m_chooser = SendableChooser()
        self.auto_names = ["Test", "TestChoreo", "Baseline", "CheckDrivetrain", "FourNote", "IntakeSetup"]
        self.m_chooser.setDefaultOption("DoNothing", "DoNothing")
        for x in self.auto_names:
            self.m_chooser.addOption(x, x)
        SmartDashboard.putData("Auto Select", self.m_chooser)

    def configureTriggersDefault(self) -> None:
        """Used to set up any commands that trigger when a measured event occurs."""
        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(  # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self.driver_controller.get_axis("LY", 0.05) * self._max_speed)
                    .with_velocity_y(-self.driver_controller.get_axis("LX", 0.05) * self._max_speed)
                    .with_rotational_rate(-self.driver_controller.get_axis("RY", 0.05) * self._max_angular_rate)
                )
            )
        )

        # Slow Mode
        button.Trigger(lambda: self.driver_controller.get_trigger("R", 0.1)).whileTrue(
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self.driver_controller.get_axis("LY", 0.05) * self._max_speed *
                        self.driver_controller.refine_trigger("R", 0.1, 0.9, 0.5))
                    .with_velocity_y(-self.driver_controller.get_axis("LX", 0.05) *
                                     self._max_speed * self.driver_controller.refine_trigger("R",
                                                                                             0.1,
                                                                                             0.9,
                                                                                             0.5))
                    .with_rotational_rate(-self.driver_controller.get_axis("RY", 0.05) *
                                          self._max_angular_rate * self.driver_controller.refine_trigger("R",
                                                                                                         0.1,
                                                                                                         0.9,
                                                                                                         0.5))
                )
            )
        )

        # Brake
        # button.Trigger(lambda: self.driver_controller.get_button("A") and not self.test_bindings).whileTrue(
        #     self.drivetrain.apply_request(lambda: self._brake))

        # Auto-alignment LEDs testing.
        button.Trigger(lambda: self.driver_controller.get_button("A") and not self.test_bindings).toggleOnTrue(
            ParallelCommandGroup(
                AlignmentLEDs(self.leds, self.drivetrain),
                self.drivetrain.apply_request(
                    lambda: (
                        self._hold_heading.with_velocity_x(
                            -self.driver_controller.get_axis("LY", 0.05) * self._max_speed)
                        .with_velocity_y(-self.driver_controller.get_axis("LX", 0.05) * self._max_speed)
                        .with_target_direction(Rotation2d.fromDegrees(180 +
                                                                      self.drivetrain.get_auto_lookahead_heading(
                                                                          [16.5, 5.53], 0.3)))))))

        # VIEW toggles on "snap heading" mode, where the driver can snap the right joystick in the direction they want
        # the robot to face.
        button.Trigger(lambda: self.driver_controller.get_button("VIEW") and not self.test_bindings).toggleOnTrue(
            self.drivetrain.apply_request(
                lambda: (
                    self._hold_heading.with_velocity_x(
                        -self.driver_controller.get_axis("LY", 0.05) * self._max_speed)
                    .with_velocity_y(-self.driver_controller.get_axis("LX", 0.05) * self._max_speed)
                    .with_target_direction(Rotation2d.fromDegrees(self.driver_controller.dir_est_ctrl("R"))))))

        # reset the field-centric heading on Y
        button.Trigger(lambda: self.driver_controller.get_button("Y") and not self.test_bindings)\
            .onTrue(self.drivetrain.runOnce(
                lambda: self.drivetrain.seed_field_relative()))

        # Play affirmative sound on Krakens.
        # button.Trigger(lambda: self.driver_controller.get_button("LB") and not self.test_bindings).onTrue(
        #     SequentialCommandGroup(
        #         runOnce(lambda: self.drivetrain.load_sound("affirmative"), self.drivetrain),
        #         runOnce(lambda: self.drivetrain.play_sound(), self.drivetrain)))

        # Pathfind to the speaker
        button.Trigger(lambda: self.driver_controller.get_d_pad_pull("E") and not self.test_bindings and
                       DriverStation.getAlliance() == DriverStation.Alliance.kBlue).onTrue(
            SequentialCommandGroup(
                runOnce(lambda: self.leds.set_state("flames"), self.leds),
                self.drivetrain.pathfind_to_pose([1.5, 5.53, 180]),
                runOnce(lambda: self.leds.set_state("default"), self.leds)))
        button.Trigger(lambda: self.driver_controller.get_d_pad_pull("E") and not self.test_bindings and
                       DriverStation.getAlliance() == DriverStation.Alliance.kRed).onTrue(
            SequentialCommandGroup(
                runOnce(lambda: self.leds.set_state("flames"), self.leds),
                self.drivetrain.pathfind_to_pose([15, 5.53, 0]),
                runOnce(lambda: self.leds.set_state("default"), self.leds)))

        # Pathfind to the amp
        button.Trigger(lambda: self.driver_controller.get_d_pad_pull("W") and not self.test_bindings and
                       DriverStation.getAlliance() == DriverStation.Alliance.kBlue).onTrue(
            SequentialCommandGroup(
                runOnce(lambda: self.leds.set_state("flames"), self.leds),
                self.drivetrain.pathfind_to_pose([1.86, 7.64, 90]),
                runOnce(lambda: self.leds.set_state("default"), self.leds)))
        button.Trigger(lambda: self.driver_controller.get_d_pad_pull("W") and not self.test_bindings and
                       DriverStation.getAlliance() == DriverStation.Alliance.kRed).onTrue(
            SequentialCommandGroup(
                runOnce(lambda: self.leds.set_state("flames"), self.leds),
                self.drivetrain.pathfind_to_pose([14.71, 7.64, 90]),
                runOnce(lambda: self.leds.set_state("default"), self.leds)))

        # Pathfind to the source
        button.Trigger(lambda: self.driver_controller.get_button("B") and not self.test_bindings and
                       DriverStation.getAlliance() == DriverStation.Alliance.kBlue).onTrue(
            SequentialCommandGroup(
                runOnce(lambda: self.leds.set_state("flames"), self.leds),
                self.drivetrain.pathfind_to_pose([15, 1, 0]),
                runOnce(lambda: self.leds.set_state("default"), self.leds)))
        button.Trigger(lambda: self.driver_controller.get_button("B") and not self.test_bindings and
                       DriverStation.getAlliance() == DriverStation.Alliance.kRed).onTrue(
            SequentialCommandGroup(
                runOnce(lambda: self.leds.set_state("flames"), self.leds),
                self.drivetrain.pathfind_to_pose([1.5, 1, 180]),
                runOnce(lambda: self.leds.set_state("default"), self.leds)))

        # Arm manual controls.
        button.Trigger(lambda: self.driver_controller.get_d_pad_pull("N") and not self.test_bindings).whileTrue(
            run(lambda: self.elevator.set_voltage_direct(1), self.elevator)).onFalse(
            run(lambda: self.elevator.set_voltage_direct(0), self.elevator))
        button.Trigger(lambda: self.driver_controller.get_d_pad_pull("S") and not self.test_bindings).whileTrue(
            run(lambda: self.elevator.set_voltage_direct(-1), self.elevator)).onFalse(
            run(lambda: self.elevator.set_voltage_direct(0), self.elevator))

        # Arm automatic controls.
        button.Trigger(lambda: self.driver_controller.get_trigger("L", 0.1)).onTrue(
            runOnce(lambda: self.elevator.set_state("stow"), self.elevator))
        button.Trigger(lambda: self.driver_controller.get_button("LB")).onTrue(
            runOnce(lambda: self.elevator.set_state("max"), self.elevator))

        # Configuration for telemetry.
        if utils.is_simulation():
            self.drivetrain.seed_field_relative(Pose2d())
        self.drivetrain.register_telemetry(lambda state: self._logger.telemeterize(state))

    def getAutonomousCommand(self) -> Command:
        """Use this to pass the autonomous command to the main Robot class.
        Returns the command to run in autonomous
        """
        if self.m_chooser.getSelected() == "DoNothing":
            return None
        else:
            selected_auto = None
            for y in self.auto_names:
                if self.m_chooser.getSelected() == y:
                    try:
                        selected_auto = PathPlannerAuto(y)
                    except FileNotFoundError:
                        selected_auto = None
            return selected_auto

    def configureTestBindings(self) -> None:
        self.configureSYSID()

        # Point all modules in a direction
        button.Trigger(lambda: self.driver_controller.get_button("MENU") and self.test_bindings) \
            .whileTrue(self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self.driver_controller.get_axis("LY", 0.05),
                               -self.driver_controller.get_axis("LX", 0.05)))
            ))

    def configureSYSID(self) -> None:
        # Run Quasistatic Translational test.
        button.Trigger(lambda: self.driver_controller.get_button("Y") and
                       self.test_bindings and
                       self.driver_controller.get_trigger("R", 0.5)).whileTrue(
            self.drivetrain.sys_id_translation_quasistatic(sysid.SysIdRoutine.Direction.kForward))
        button.Trigger(lambda: self.driver_controller.get_button("B") and
                       self.test_bindings and
                       self.driver_controller.get_trigger("R", 0.5)).whileTrue(
            self.drivetrain.sys_id_translation_quasistatic(sysid.SysIdRoutine.Direction.kReverse))
        # Run Dynamic Translational test.
        button.Trigger(lambda: self.driver_controller.get_button("A") and
                       self.test_bindings and
                       self.driver_controller.get_trigger("R", 0.5)).whileTrue(
            self.drivetrain.sys_id_translation_dynamic(sysid.SysIdRoutine.Direction.kForward))
        button.Trigger(lambda: self.driver_controller.get_button("X") and
                       self.test_bindings and
                       self.driver_controller.get_trigger("R", 0.5)).whileTrue(
            self.drivetrain.sys_id_translation_dynamic(sysid.SysIdRoutine.Direction.kReverse))
        # Run Quasistatic Rotational test.
        button.Trigger(lambda: self.driver_controller.get_button("Y") and
                       self.test_bindings and
                       self.driver_controller.get_button("RB")).whileTrue(
            self.drivetrain.sys_id_rotation_quasistatic(sysid.SysIdRoutine.Direction.kForward))
        button.Trigger(lambda: self.driver_controller.get_button("B") and
                       self.test_bindings and
                       self.driver_controller.get_button("RB")).whileTrue(
            self.drivetrain.sys_id_rotation_quasistatic(sysid.SysIdRoutine.Direction.kReverse))
        # Run Dynamic Translational test.
        button.Trigger(lambda: self.driver_controller.get_button("A") and
                       self.test_bindings and
                       self.driver_controller.get_button("RB")).whileTrue(
            self.drivetrain.sys_id_rotation_dynamic(sysid.SysIdRoutine.Direction.kForward))
        button.Trigger(lambda: self.driver_controller.get_button("X") and
                       self.test_bindings and
                       self.driver_controller.get_button("RB")).whileTrue(
            self.drivetrain.sys_id_rotation_dynamic(sysid.SysIdRoutine.Direction.kReverse))
        # Run Quasistatic Steer test.
        button.Trigger(lambda: self.driver_controller.get_button("Y") and
                       self.test_bindings and
                       self.driver_controller.get_button("LB")).whileTrue(
            self.drivetrain.sys_id_steer_quasistatic(sysid.SysIdRoutine.Direction.kForward))
        button.Trigger(lambda: self.driver_controller.get_button("B") and
                       self.test_bindings and
                       self.driver_controller.get_button("LB")).whileTrue(
            self.drivetrain.sys_id_steer_quasistatic(sysid.SysIdRoutine.Direction.kReverse))
        # Run Dynamic Translational test.
        button.Trigger(lambda: self.driver_controller.get_button("A") and
                       self.test_bindings and
                       self.driver_controller.get_button("LB")).whileTrue(
            self.drivetrain.sys_id_steer_dynamic(sysid.SysIdRoutine.Direction.kForward))
        button.Trigger(lambda: self.driver_controller.get_button("X") and
                       self.test_bindings and
                       self.driver_controller.get_button("LB")).whileTrue(
            self.drivetrain.sys_id_steer_dynamic(sysid.SysIdRoutine.Direction.kReverse))

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
        NamedCommands.registerCommand("override_heading",
                                      runOnce(lambda: self.drivetrain.set_pathplanner_rotation_override(True)))
        NamedCommands.registerCommand("disable_override_heading",
                                      runOnce(lambda: self.drivetrain.set_pathplanner_rotation_override(False)))
