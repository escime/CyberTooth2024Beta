import math

from commands2 import Command, Subsystem, sysid
from phoenix6 import swerve, units, utils, SignalLogger, orchestra
from typing import Callable, overload
from wpilib import DriverStation, Notifier, RobotController, SmartDashboard, Field2d
# from wpiutil import Sendable, SendableBuilder
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import PIDConstants, RobotConfig
from pathplannerlib.path import PathConstraints
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.commands import PathfindingCommand
from constants import AutoConstants
from ntcore import NetworkTableInstance
# import types
from helpers.data_wrap import DataWrap


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

    auto_request = swerve.requests.ApplyChassisSpeeds().with_drive_request_type(
        swerve.SwerveModule.DriveRequestType.VELOCITY)

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

        self.config = RobotConfig.fromGUISettings()

        self._sim_notifier: Notifier | None = None
        self._last_sim_time: units.second = 0.0

        self._has_applied_operator_perspective = False
        """Keep track if we've ever applied the operator perspective before or not"""

        if utils.is_simulation():
            self._start_sim_thread()

        self.odo_ll_table = NetworkTableInstance.getDefault().getTable("limelight")
        self.gp_ll_table = NetworkTableInstance.getDefault().getTable("limelight-gp")
        self.gp_ll_gp_mode = True
        self.tx = 0.0
        self.gp_locations = []
        self.gp_field = Field2d()

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
        self.vx_robot_new = 0
        self.vy_robot_new = 0
        self.omega_robot_new = 0
        self.vx_robot_old = 0
        self.vy_robot_old = 0
        self.omega_robot_old = 0
        self.ax_robot = 0
        self.ay_robot = 0
        self.alpha_robot = 0

        # Setup Orchestra.
        self.orchestra = orchestra.Orchestra()
        for i in range(0, 4):
            self.orchestra.add_instrument(self.modules[i].drive_motor)
            self.orchestra.add_instrument(self.modules[i].steer_motor)
        # self.orchestra.load_music("affirmative.chrp")

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

        SmartDashboard.putBoolean("Orchestra Active", self.orchestra.is_playing())

    def vel_acc_periodic(self) -> None:
        """Calculates the instantaneous robot velocity and acceleration."""
        self.vx_new, self.vy_new, self.omega_new = self.get_field_relative_velocity()
        self.vx_robot_new, self.vy_robot_new, self.omega_robot_new = self.get_robot_relative_velocity()
        self.ax, self.ay, self.alpha = self.get_field_relative_acceleration([self.vx_new, self.vy_new, self.omega_new],
                                                                            [self.vx_old, self.vy_old, self.omega_old],
                                                                            utils.get_current_time_seconds() - self.loop_time)
        self.ax_robot, self.ay_robot, self.alpha_robot = (
            self.get_field_relative_acceleration([self.vx_robot_new, self.vy_robot_old, self.omega_robot_new],
                                                 [self.vx_robot_old, self.vy_robot_old, self.omega_robot_old],
                                                 utils.get_current_time_seconds() - self.loop_time))

        self.vx_old = self.vx_new
        self.vy_old = self.vy_new
        self.omega_old = self.omega_new
        self.vx_robot_old = self.vx_robot_new
        self.vy_robot_old = self.vy_robot_new
        self.omega_robot_old = self.omega_robot_new

        self.loop_time = utils.get_current_time_seconds()
        SmartDashboard.putNumber("Robot Linear Speed", math.sqrt((self.vx_new * self.vx_new) + (self.vy_new * self.vy_new)))
        SmartDashboard.putBoolean("Robot Slipping X", self.get_slip_detected()[0])
        SmartDashboard.putBoolean("Robot Slipping Y", self.get_slip_detected()[1])
        self.get_slip_detected()

    def limelight_periodic(self) -> None:
        """Fuses limelight data with the pose estimator."""
        self.odo_ll_table.putNumberArray("robot_orientation_set",
                                         [self.get_pose().rotation().degrees(), 0, 0, 0, 0, 0])

        odo_ll_botpose = self.odo_ll_table.getEntry("botpose_orb_wpiblue").getDoubleArray([0.0, 0.0, 0.0, 0.0,
                                                                                           0.0, 0.0, 0.0, 0.0,
                                                                                           0.0, 0.0, 0.0, 0.0])
        if not self.gp_ll_gp_mode:
            self.gp_ll_table.putNumberArray("robot_orientation_set",
                                            [self.get_pose().rotation().degrees(), 0, 0, 0, 0, 0])

            gp_ll_botpose = self.gp_ll_table.getEntry("botpose_orb_wpiblue").getDoubleArray([0.0, 0.0, 0.0, 0.0,
                                                                                             0.0, 0.0, 0.0, 0.0,
                                                                                             0.0, 0.0, 0.0, 0.0])
            if odo_ll_botpose[9] < gp_ll_botpose[9] < 10 and odo_ll_botpose[7] >= 1:
                self.add_vision_measurement(Pose2d(Translation2d(odo_ll_botpose[0], odo_ll_botpose[1]),
                                                   Rotation2d.fromDegrees((odo_ll_botpose[5] + 360) % 360)),
                                            utils.get_current_time_seconds() - (odo_ll_botpose[6]))
            elif gp_ll_botpose[9] < odo_ll_botpose[9] < 10 and gp_ll_botpose[7] >= 1:
                self.add_vision_measurement(Pose2d(Translation2d(gp_ll_botpose[0], gp_ll_botpose[1]),
                                                   Rotation2d.fromDegrees((gp_ll_botpose[5] + 360) % 360)),
                                            utils.get_current_time_seconds() - (gp_ll_botpose[6]))
        else:
            if odo_ll_botpose[9] < 10 and odo_ll_botpose[7] >= 1:
                SmartDashboard.putBoolean("Valid AprilTag Detected?", True)
                self.add_vision_measurement(Pose2d(Translation2d(odo_ll_botpose[0], odo_ll_botpose[1]),
                                                   Rotation2d.fromDegrees((odo_ll_botpose[5] + 360) % 360)),
                                            utils.get_current_time_seconds() - (odo_ll_botpose[6] / 1000.0),
                                            (0.2, 0.2, 999999999))
            else:
                SmartDashboard.putBoolean("Valid AprilTag Detected?", False)
            if self.gp_ll_table.getEntry("tv").getDouble(-1) == 1:
                self.tx = self.gp_ll_table.getEntry("tx").getDouble(0)
                self.add_gp_to_detected_list(self.estimate_gp_location())
                SmartDashboard.putString("GP Locations", str(self.gp_locations))
            else:
                self.remove_gp_from_detected_list()
                SmartDashboard.putString("GP Locations", str(self.gp_locations))

    def estimate_gp_location(self) -> [float, float, float]:
        h = (self.gp_ll_table.getEntry("ta").getDouble(0) * 0.01 * 788.644 + 1.692) * 0.0254  # 1 replaced with empirical ratio
        y = h * math.sin(self.get_pose().rotation().radians())
        x = h * math.cos(self.get_pose().rotation().radians())

        return [self.get_pose().x + x, self.get_pose().y + y, self.get_pose().rotation().degrees(),
                utils.get_current_time_seconds()]

    def add_gp_to_detected_list(self, gp: [float, float, float, float]) -> None:
        if self.gp_locations:
            for x in self.gp_locations:
                if gp[0] - 0.5 < x[0] < gp[0] + 0.5 and gp[1] - 0.5 < x[1] < gp[1] + 0.5:
                    x[0] = gp[0]
                    x[1] = gp[1]
                    x[2] = gp[2]
                    x[3] = gp[3]
                else:
                    self.gp_locations.append([gp[0], gp[1], gp[2], gp[3]])
        else:
            self.gp_locations.append([gp[0], gp[1], gp[2], gp[3]])
        # self.display_gps_as_poses()

    def remove_gp_from_detected_list(self):
        for x in self.gp_locations:
            if (self.get_pose().rotation().degrees() - 2 < x[2] < self.get_pose().rotation().degrees() + 2 or
                    x[3] < utils.get_current_time_seconds() - 5):
                self.gp_locations.remove(x)
                self.display_gps_as_poses()

    def display_gps_as_poses(self):
        for i in range(0, len(self.gp_locations)):
            self.gp_field.getObject("gp" + str(i)).setPose(Pose2d(self.gp_locations[i][0], self.gp_locations[i][1], 0))
        SmartDashboard.putData(self.gp_field)

    def get_field_relative_velocity(self) -> [float, float, float]:
        """Returns the instantaneous velocity of the robot."""
        return self.get_chassis_speeds().vx * self.get_pose().rotation().cos() - \
            self.get_chassis_speeds().vy * self.get_pose().rotation().sin(), \
            self.get_chassis_speeds().vy * self.get_pose().rotation().cos() + \
            self.get_chassis_speeds().vx * self.get_pose().rotation().sin(), self.get_chassis_speeds().omega

    def get_robot_relative_velocity(self) -> [float, float, float]:
        return self.get_chassis_speeds().vx, self.get_chassis_speeds().vy, self.get_chassis_speeds().omega

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
        AutoBuilder.configure(
            self.get_pose,
            self.seed_field_relative,
            self.get_chassis_speeds,
            lambda speeds, feedforwards: self.set_control(self.auto_request.with_speeds(speeds)),
            PPHolonomicDriveController(
                PIDConstants(AutoConstants.x_pid[0], AutoConstants.x_pid[1], AutoConstants.x_pid[2]),
                PIDConstants(AutoConstants.y_pid[0], AutoConstants.y_pid[1], AutoConstants.y_pid[2]),
                AutoConstants.speed_at_12_volts,
            ),
            self.config,
            self.get_path_flip,
            self
        )

        PPHolonomicDriveController.setRotationTargetOverride(self.pathplanner_rotation_override)

    def pathplanner_rotation_override(self) -> Rotation2d:
        """Provides the overridden heading in the event the override has been toggled. Returns None if override is
        disabled, which is the default."""
        if self.pathplanner_rotation_overridden == "goal":
            return Rotation2d.fromDegrees(self.get_goal_alignment_heading())
        elif self.pathplanner_rotation_overridden == "gp":
            return Rotation2d.fromDegrees(self.get_gp_alignment_heading())
        else:
            return None

    def get_gp_alignment_heading(self) -> float:
        return self.get_pose().rotation().degrees() + self.tx

    def get_goal_alignment_heading(self) -> float:
        """Returns the required target heading to point at a goal."""
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            return self.get_auto_lookahead_heading([16.5, 5.53], 0.3)
        else:
            return self.get_auto_lookahead_heading([0, 5.53], 0.3)

    def set_pathplanner_rotation_override(self, override: str) -> None:
        """Sets whether pathplanner uses an alternate heading controller."""
        self.pathplanner_rotation_overridden = override

    def load_sound(self, sound: str) -> None:
        """Prepares drivetrain Krakens to play a sound."""
        for i in range(0, 4):
            self.orchestra.add_instrument(self.modules[i].drive_motor)
            self.orchestra.add_instrument(self.modules[i].steer_motor)
        if sound == "affirmative":
            self.orchestra.load_music("meme1.chrp")
        elif sound == "negative":
            self.orchestra.load_music("negative.chrp")
        else:
            self.orchestra.load_music("invalid_sound.chrp")
        self.orchestra.play()

    def clear_orchestra(self) -> None:
        self.orchestra.stop()
        self.orchestra.clear_instruments()

    def get_gp_in_view(self) -> bool:
        return self.gp_ll_table.getEntry("tv").getDouble(-1) == 1

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

        return AutoBuilder.pathfindToPose(
            target_pose,
            constraints,
            goal_end_vel=0.0
        )

    def get_close_to_target(self, target: [float, float], good_range: float) -> bool:
        pose = [self.get_pose().x, self.get_pose().y]
        c = math.sqrt(((target[0] - pose[0]) * (target[0] - pose[0])) + ((target[1] - pose[1]) * (target[1] - pose[1])))
        return good_range >= c

    def get_inertial_acceleration(self) -> [float, float]:
        return [self.pigeon2.get_acceleration_x(True).value_as_double * 9.81,
                self.pigeon2.get_acceleration_y(True).value_as_double * 9.81]

    def get_slip_detected(self) -> [bool, bool]:
        x_slip = False
        y_slip = False
        SmartDashboard.putNumber("IMU Acceleration X", self.get_inertial_acceleration()[0])
        SmartDashboard.putNumber("Target Acceleration X", self.ax_robot)
        if (self.get_inertial_acceleration()[0] > 0 and self.ax_robot > 0 and
                self.get_inertial_acceleration()[0] > self.ax_robot + 2):
            x_slip = True
        elif (self.get_inertial_acceleration()[0] < 0 and self.ax_robot < 0 and
              self.get_inertial_acceleration()[0] < self.ax_robot - 2):
            x_slip = True

        if (self.get_inertial_acceleration()[1] > 0 and self.ay_robot > 0 and
                self.get_inertial_acceleration()[1] > self.ay_robot + 2):
            y_slip = True
        elif (self.get_inertial_acceleration()[1] < 0 and self.ay_robot < 0 and
              self.get_inertial_acceleration()[1] < self.ay_robot - 2):
            y_slip = True

        return [x_slip, y_slip]
