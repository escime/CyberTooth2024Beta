import math

from commands2 import Command, Subsystem, sysid
from phoenix6 import swerve, units, utils, SignalLogger, orchestra
from typing import Callable, overload
from wpilib import DriverStation, Notifier, RobotController
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from pathplannerlib.auto import AutoBuilder, HolonomicPathFollowerConfig, ReplanningConfig
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState
from pathplannerlib.commands import PathfindHolonomic
from constants import AutoConstants
from ntcore import NetworkTableInstance


class CommandSwerveDrivetrain(Subsystem, swerve.SwerveDrivetrain):
    """
    Class that extends the Phoenix 6 SwerveDrivetrain class and implements
    Subsystem so it can easily be used in command-based projects.
    """

    _SIM_LOOP_PERIOD: units.second = 0.005

    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    """Blue alliance sees forward as 0 degrees (toward red alliance wall)"""
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)
    """Red alliance sees forward as 180 degrees (toward blue alliance wall)"""

    auto_request = swerve.requests.ApplyChassisSpeeds()

    @overload
    def __init__(self, drivetrain_constants: swerve.SwerveDrivetrainConstants,
                 modules: list[swerve.SwerveModuleConstants]) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so user should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param driveTrainConstants: Drivetrain-wide constants for the swerve drive
        :type driveTrainConstants:  swerve.SwerveDrivetrainConstants
        :param modules:             Constants for each specific module
        :type modules:              list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(self, drivetrain_constants: swerve.SwerveDrivetrainConstants, odometry_update_frequency: units.hertz,
                 modules: list[swerve.SwerveModuleConstants]) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so user should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param driveTrainConstants:         Drivetrain-wide constants for the swerve drive
        :type driveTrainConstants:          swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
            self,
            drivetrain_constants: swerve.SwerveDrivetrainConstants,
            odometry_update_frequency: units.hertz,
            odometry_standard_deviation: tuple[float, float, float],
            vision_standard_deviation: tuple[float, float, float],
            modules: list[swerve.SwerveModuleConstants]
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so user should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param driveTrainConstants:         Drivetrain-wide constants for the swerve drive
        :type driveTrainConstants:          swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param odometry_standard_deviation: The standard deviation for odometry calculation
        :type odometry_standard_deviation:  tuple[float, float, float]
        :param vision_standard_deviation:   The standard deviation for vision calculation
        :type vision_standard_deviation:    tuple[float, float, float]
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    def __init__(
            self,
            drivetrain_constants: swerve.SwerveDrivetrainConstants,
            arg2=None,
            arg3=None,
            arg4=None,
            arg5=None,
    ):
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(self, drivetrain_constants, arg2, arg3, arg4, arg5)

        self._sim_notifier: Notifier | None = None
        self._last_sim_time: units.second = 0.0

        self._has_applied_operator_perspective = False
        """Keep track if we've ever applied the operator perspective before or not"""

        if utils.is_simulation():
            self._start_sim_thread()

        self.odo_ll_table = NetworkTableInstance.getDefault().getTable("limelight")
        self.gp_ll_table = NetworkTableInstance.getDefault().getTable("limelight-gp")
        self.gp_ll_gp_mode = True
        self.tx = 0

        self.pathplanner_rotation_overridden = False
        self.configure_pathplanner()

        # Setup for velocity and acceleration calculations.
        self.loop_time = utils.get_current_time_seconds()
        self.vx_old = 0
        self.vy_old = 0
        self.omega_old = 0
        self.vx_new = 0
        self.vy_new = 0
        self.omega_new = 0
        self.ax = 0
        self.ay = 0
        self.alpha = 0

        # Setup Orchestra.
        self.orchestra = orchestra.Orchestra()
        for i in range(0, 4):
            self.orchestra.add_instrument(self.modules[i].drive_motor)
            self.orchestra.add_instrument(self.modules[i].steer_motor)
        self.orchestra.load_music("affirmative.chrp")

        # Setup SYSID Routines.
        self.sys_id_routine_translation = sysid.SysIdRoutine(
            sysid.SysIdRoutine.Config(
                stepVoltage=4.0,
                recordState=lambda state: SignalLogger.write_string("state", SysIdRoutineLog.stateEnumToString(state))
            ),
            sysid.SysIdRoutine.Mechanism(
                lambda volts: self.set_control(swerve.requests.SysIdSwerveTranslation().with_volts(volts)),
                lambda log: None,
                self
            )
        )
        self.sys_id_routine_rotation = sysid.SysIdRoutine(
            sysid.SysIdRoutine.Config(
                stepVoltage=4.0,
                recordState=lambda state: SignalLogger.write_string("state", SysIdRoutineLog.stateEnumToString(state))
            ),
            sysid.SysIdRoutine.Mechanism(
                lambda volts: self.set_control(swerve.requests.SysIdSwerveRotation().with_volts(volts)),
                lambda log: None,
                self
            )
        )
        self.sys_id_routine_steer = sysid.SysIdRoutine(
            sysid.SysIdRoutine.Config(
                stepVoltage=4.0,
                recordState=lambda state: SignalLogger.write_string("state", SysIdRoutineLog.stateEnumToString(state))
            ),
            sysid.SysIdRoutine.Mechanism(
                lambda volts: self.set_control(swerve.requests.SysIdSwerveSteerGains().with_volts(volts)),
                lambda log: None,
                self
            )
        )

    def apply_request(self, request: Callable[[], swerve.requests.SwerveRequest]) -> Command:
        """
        Returns a command that applies the specified control request to this swerve drivetrain.

        :param request: Lambda returning the request to apply
        :type request: Callable[[], swerve.requests.SwerveRequest]
        :returns: Command to run
        :rtype: Command
        """
        return self.run(lambda: self.set_control(request()))

    def periodic(self):
        # Periodically try to apply the operator perspective.
        # If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
        # This allows us to correct the perspective in case the robot code restarts mid-match.
        # Otherwise, only check and apply the operator perspective if the DS is disabled.
        # This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
        if not self._has_applied_operator_perspective or DriverStation.isDisabled():
            alliance_color = DriverStation.getAlliance()
            if alliance_color is not None:
                self.set_operator_perspective_forward(
                    self._RED_ALLIANCE_PERSPECTIVE_ROTATION
                    if alliance_color == DriverStation.Alliance.kRed
                    else self._BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self._has_applied_operator_perspective = True

        # Configure limelight settings and data transfer.
        self.limelight_periodic()

        # Update robot velocity and acceleration.
        self.vel_acc_periodic()

    def vel_acc_periodic(self) -> None:
        """Calculates the instantaneous robot velocity and acceleration."""
        self.vx_new, self.vy_new, self.omega_new = self.get_field_relative_velocity()
        self.ax, self.ay, self.alpha = self.get_field_relative_acceleration([self.vx_new, self.vy_new, self.omega_new],
                                                                            [self.vx_old, self.vy_old, self.omega_old],
                                                                            utils.get_current_time_seconds() - self.loop_time)
        self.vx_old = self.vx_new
        self.vy_old = self.vy_new
        self.omega_old = self.omega_new
        self.loop_time = utils.get_current_time_seconds()

    def limelight_periodic(self) -> None:
        """Fuses limelight data with the pose estimator."""
        self.odo_ll_table.putNumberArray("robot_orientation_set",
                                         [self.get_pose().rotation().degrees(), 0, 0, 0, 0, 0])
        if not self.gp_ll_gp_mode:
            self.gp_ll_table.putNumberArray("robot_orientation_set",
                                            [self.get_pose().rotation().degrees(), 0, 0, 0, 0, 0])

        odo_ll_botpose = self.odo_ll_table.getEntry("botpose_orb_wpiblue").getDoubleArray([0.0, 0.0, 0.0, 0.0,
                                                                                           0.0, 0.0, 0.0, 0.0,
                                                                                           0.0, 0.0, 0.0, 0.0])
        if not self.gp_ll_gp_mode:
            gp_ll_botpose = self.gp_ll_table.getEntry("botpose_orb_wpiblue").getDoubleArray([0.0, 0.0, 0.0, 0.0,
                                                                                             0.0, 0.0, 0.0, 0.0,
                                                                                             0.0, 0.0, 0.0, 0.0])
        if not self.gp_ll_gp_mode:
            if odo_ll_botpose[9] < gp_ll_botpose[9] < 20 and odo_ll_botpose[7] >= 1:
                self.add_vision_measurement(Pose2d(Translation2d(odo_ll_botpose[0], odo_ll_botpose[1]),
                                                   Rotation2d.fromDegrees((odo_ll_botpose[5] + 360) % 360)),
                                            utils.get_current_time_seconds() - (odo_ll_botpose[6]))
            elif gp_ll_botpose[9] < odo_ll_botpose[9] < 10 and gp_ll_botpose[7] >= 1:
                self.add_vision_measurement(Pose2d(Translation2d(gp_ll_botpose[0], gp_ll_botpose[1]),
                                                   Rotation2d.fromDegrees((gp_ll_botpose[5] + 360) % 360)),
                                            utils.get_current_time_seconds() - (gp_ll_botpose[6]))
        else:
            if odo_ll_botpose[9] and odo_ll_botpose[7] >= 1:
                self.add_vision_measurement(Pose2d(Translation2d(odo_ll_botpose[0], odo_ll_botpose[1]),
                                                   Rotation2d.fromDegrees((odo_ll_botpose[5] + 360) % 360)),
                                            utils.get_current_time_seconds() - (odo_ll_botpose[6]))
            if self.gp_ll_table.getEntry("tv").getDouble(-1) == 1:
                self.tx = self.gp_ll_table.getEntry("tx").getDouble(0)

    def get_field_relative_velocity(self) -> [float, float]:
        """Returns the instantaneous velocity of the robot."""
        return self.get_chassis_speeds().vx * self.get_pose().rotation().cos() - \
            self.get_chassis_speeds().vy * self.get_pose().rotation().sin(), \
            self.get_chassis_speeds().vy * self.get_pose().rotation().cos() + \
            self.get_chassis_speeds().vx * self.get_pose().rotation().sin(), self.get_chassis_speeds().omega

    def get_angular_velocity(self) -> float:
        """Returns the instantaneous angular velocity of the robot."""
        return self.get_chassis_speeds().omega

    def get_field_relative_acceleration(self, new_speed, old_speed, time: float) -> [float, float, float]:
        """Returns the instantaneous acceleration of the robot."""
        ax = (new_speed[0] - old_speed[0]) / time
        ay = (new_speed[1] - old_speed[1]) / time
        alpha = (new_speed[2] - old_speed[2]) / time

        if abs(ax) > 6.0:
            ax = 6.0 * math.copysign(1, ax)
        if abs(ay) > 6.0:
            ay = 6.0 * math.copysign(1, ay)
        if abs(alpha) > 4 * math.pi:
            alpha = 4 * math.pi * math.copysign(1, alpha)

        return ax, ay, alpha

    def _start_sim_thread(self):
        def _sim_periodic():
            current_time = utils.get_current_time_seconds()
            delta_time = current_time - self._last_sim_time
            self._last_sim_time = current_time

            # use the measured time delta, get battery voltage from WPILib
            self.update_sim_state(delta_time, RobotController.getBatteryVoltage())

        self._last_sim_time = utils.get_current_time_seconds()
        self._sim_notifier = Notifier(_sim_periodic)
        self._sim_notifier.startPeriodic(self._SIM_LOOP_PERIOD)

    def sys_id_translation_quasistatic(self, direction: sysid.SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_translation.quasistatic(direction)

    def sys_id_translation_dynamic(self, direction: sysid.SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_translation.dynamic(direction)

    def sys_id_rotation_quasistatic(self, direction: sysid.SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_rotation.quasistatic(direction)

    def sys_id_rotation_dynamic(self, direction: sysid.SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_rotation.dynamic(direction)

    def sys_id_steer_quasistatic(self, direction: sysid.SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_steer.quasistatic(direction)

    def sys_id_steer_dynamic(self, direction: sysid.SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_steer.dynamic(direction)

    def get_chassis_speeds(self):
        return AutoConstants.kinematics.toChassisSpeeds(self.get_state().module_states)

    def get_path_flip(self) -> bool:
        """Tells PathPlanner whether to flip the path for autonomous."""
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            return True
        else:
            return False

    def get_pose(self) -> Pose2d:
        """Returns the robot pose."""
        return self.get_state().pose

    def configure_pathplanner(self) -> None:
        """Configures all pathplanner settings."""
        AutoBuilder.configureHolonomic(
            self.get_pose,
            self.seed_field_relative,
            self.get_chassis_speeds,
            lambda speeds: self.set_control(self.auto_request.with_speeds(speeds)),
            HolonomicPathFollowerConfig(
                PIDConstants(AutoConstants.x_pid[0], AutoConstants.x_pid[1], AutoConstants.x_pid[2]),
                PIDConstants(AutoConstants.y_pid[0], AutoConstants.y_pid[1], AutoConstants.y_pid[2]),
                AutoConstants.speed_at_12_volts,
                AutoConstants.drive_base_radius,
                ReplanningConfig()
            ),
            self.get_path_flip,
            self
        )

        PPHolonomicDriveController.setRotationTargetOverride(self.pathplanner_rotation_override)

    def pathplanner_rotation_override(self) -> Rotation2d:
        """Provides the overridden heading in the event the override has been toggled. Returns None if override is
        disabled, which is the default."""
        if self.pathplanner_rotation_overridden:
            return Rotation2d.fromDegrees(self.get_goal_alignment_heading())
        else:
            return None

    def get_goal_alignment_heading(self) -> float:
        """Returns the required target heading to point at a goal."""
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            return self.get_auto_lookahead_heading([16.5, 5.53], 0.3)
        else:
            return self.get_auto_lookahead_heading([0, 5.53], 0.3)

    def set_pathplanner_rotation_override(self, override: bool) -> None:
        """Sets whether pathplanner uses an alternate heading controller."""
        self.pathplanner_rotation_overridden = override

    def load_sound(self, sound: str) -> None:
        """Prepares drivetrain Krakens to play a sound."""
        if sound == "affirmative":
            self.orchestra.load_music("affirmative.chrp")
        elif sound == "negative":
            self.orchestra.load_music("negative.chrp")
        else:
            self.orchestra.load_music("invalid_sound.chrp")

    def play_sound(self) -> None:
        """Plays a sound on all drivetrain Krakens."""
        self.orchestra.play()

    def get_auto_target_heading(self, target: [float, float]) -> float:
        """Acquires the target heading required to point at a goal."""
        current_pose = self.get_pose()
        return math.atan2(target[1] - current_pose.y, target[0] - current_pose.x) * 180 / math.pi

    def get_auto_lookahead_heading(self, target: [float, float], time_compensation: float) -> float:
        """Acquires the target heading required to point at a goal while the robot is in motion."""
        current_pose = self.get_pose()
        adjusted_pose = Pose2d(current_pose.x + self.vx_new * time_compensation,
                               current_pose.y + self.vy_new * time_compensation,
                               current_pose.rotation() + Rotation2d(self.omega_new * time_compensation))
        return math.atan2(target[1] - adjusted_pose.y, target[0] - adjusted_pose.x) * 180 / math.pi

    def pathfind_to_pose(self, target: [float, float, float]):
        """Command for pathfinding between current pose and a target pose in teleoperated."""
        target_pose = Pose2d(target[0], target[1], Rotation2d.fromDegrees(target[2]))
        constraints = PathConstraints(4, 4, 9.424, 12.567)

        return PathfindHolonomic(
            constraints,
            self.get_pose,
            self.get_chassis_speeds,
            lambda speeds: self.set_control(self.auto_request.with_speeds(speeds)),
            HolonomicPathFollowerConfig(
                PIDConstants(AutoConstants.x_pid[0], AutoConstants.x_pid[1], AutoConstants.x_pid[2]),
                PIDConstants(AutoConstants.y_pid[0], AutoConstants.y_pid[1], AutoConstants.y_pid[2]),
                AutoConstants.speed_at_12_volts,
                AutoConstants.drive_base_radius,
                ReplanningConfig()
            ),
            lambda: False,
            self,
            rotation_delay_distance=0,
            target_pose=target_pose,
            goal_end_vel=0
        )
