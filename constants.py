"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""
from wpimath.units import inchesToMeters, lbsToKilograms
from phoenix6 import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from math import pi


class OIConstants:
    kDriverControllerPort = 0
    kOperatorControllerPort = 1


class LEDConstants:
    port = 0
    strip_length = 30


class AutoConstants:
    # Copy these values from TunerConstants.
    _front_left_x_pos: units.meter = inchesToMeters(9.625)
    _front_left_y_pos: units.meter = inchesToMeters(9.625)
    _front_right_x_pos: units.meter = inchesToMeters(9.625)
    _front_right_y_pos: units.meter = inchesToMeters(-9.625)
    _back_left_x_pos: units.meter = inchesToMeters(-9.625)
    _back_left_y_pos: units.meter = inchesToMeters(9.625)
    _back_right_x_pos: units.meter = inchesToMeters(-9.625)
    _back_right_y_pos: units.meter = inchesToMeters(-9.625)

    # Math for based on copied units from Tuner.
    _front_left_translation = Translation2d(_front_left_x_pos, _front_left_y_pos)
    _front_right_translation = Translation2d(_front_right_x_pos, _front_right_y_pos)
    _back_left_translation = Translation2d(_back_left_x_pos, _back_left_y_pos)
    _back_right_translation = Translation2d(_back_right_x_pos, _back_right_y_pos)
    kinematics = SwerveDrive4Kinematics(_front_left_translation, _front_right_translation, _back_left_translation,
                                        _back_right_translation)

    drive_base_radius = Translation2d(_front_left_x_pos, _front_left_y_pos).norm()
    speed_at_12_volts: units.meters_per_second = 4.73

    # PID Constants for PathPlanner
    x_pid = [10, 0, 0]
    y_pid = [10, 0, 0]


class ElevatorConstants:
    can_ids = [30, 31]  # The main motor should be listed first. The rest may be listed in any order.

    drum_diameter_in = 2
    drum_diameter_m = inchesToMeters(drum_diameter_in)
    state_values = {"stow": 0, "max": 60 / (drum_diameter_in * pi)}

    use_remote_sensor = True
    remote_sensor_id = 32

    stator_current_limit = 80
    use_stator_current_limit = True

    gearbox_ratio = 4

    mm_cruise_velocity = 10
    mm_acceleration = 15
    mm_jerk = 100

    kg = 0.14
    ks = 0.25
    kv = 37.6
    ka = 0
    kp = 60
    ki = 0
    kd = 0

    carriage_weight = lbsToKilograms(10)
    min_height_in = 0
    max_height_in = 60
    min_height_m = inchesToMeters(min_height_in)
    max_height_m = inchesToMeters(max_height_in)

    elevator_angle_degrees = 90

    elevator_at_target_threshold = 0.05
    elevator_upper_limit = 60 / (drum_diameter_in * pi)
    elevator_lower_limit = 0 / (drum_diameter_in * pi)
