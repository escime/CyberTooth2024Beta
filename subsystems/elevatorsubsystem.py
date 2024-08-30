from commands2 import Subsystem

from phoenix6.hardware import TalonFX
from phoenix6.controls import MotionMagicVoltage, VoltageOut, Follower
from phoenix6.configs import TalonFXConfiguration
from phoenix6.status_code import StatusCode
from phoenix6.utils import get_current_time_seconds, is_simulation

from wpilib import Mechanism2d, Color8Bit, Color, SmartDashboard
from wpilib.simulation import ElevatorSim
from wpimath.system.plant import DCMotor

from math import pi

from constants import ElevatorConstants


class ElevatorSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self._last_sim_time = get_current_time_seconds()
        self.state_values = ElevatorConstants.state_values
        self.state = "stow"
        self.last_time = get_current_time_seconds()

        self.lift_main_mm = MotionMagicVoltage(0, enable_foc=False)
        self.lift_main_vo = VoltageOut(0, enable_foc=False)

        for motor_id in ElevatorConstants.can_ids:
            motor = TalonFX(motor_id)
            motor_config = TalonFXConfiguration()

            if motor_id == ElevatorConstants.can_ids[0]:
                if ElevatorConstants.use_remote_sensor:
                    motor_config.feedback.feedback_remote_sensor_id = ElevatorConstants.remote_sensor_id
                    motor_config.feedback.feedback_sensor_source = (
                        TalonFXConfiguration().feedback.feedback_sensor_source.REMOTE_CANCODER)
                else:
                    motor.set_position(0)

            motor_config.current_limits.stator_current_limit = ElevatorConstants.stator_current_limit
            motor_config.current_limits.stator_current_limit_enable = ElevatorConstants.use_stator_current_limit
            motor_config.feedback.sensor_to_mechanism_ratio = ElevatorConstants.gearbox_ratio

            motor_mm_config = motor_config.motion_magic
            motor_mm_config.motion_magic_cruise_velocity = ElevatorConstants.mm_cruise_velocity
            motor_mm_config.motion_magic_acceleration = ElevatorConstants.mm_acceleration
            motor_mm_config.motion_magic_jerk = ElevatorConstants.mm_jerk

            motor_slot0_config = motor_config.slot0
            motor_slot0_config.with_k_g(ElevatorConstants.kg)
            motor_slot0_config.with_k_s(ElevatorConstants.ks)
            motor_slot0_config.with_k_v(ElevatorConstants.kv)
            motor_slot0_config.with_k_a(ElevatorConstants.ka)
            motor_slot0_config.with_k_p(ElevatorConstants.kp)
            motor_slot0_config.with_k_i(ElevatorConstants.ki)
            motor_slot0_config.with_k_d(ElevatorConstants.kd)

            status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
            for _ in range(0, 5):
                status = motor.configurator.apply(motor_config)
                if status.is_ok():
                    break
            if not status.is_ok():
                print(f"Could not apply configs, error code: {status.name}")

            if motor_id != ElevatorConstants.can_ids[0]:
                follower_control_request = Follower(ElevatorConstants.can_ids[0], False)
                motor.set_control(follower_control_request)
            else:
                self.lift_main = motor

        self.lift_sim = self.lift_main.sim_state
        self.elevator_sim = ElevatorSim(
            DCMotor.krakenX60(len(ElevatorConstants.can_ids)),
            ElevatorConstants.gearbox_ratio,
            ElevatorConstants.carriage_weight,
            ElevatorConstants.drum_diameter_m,
            ElevatorConstants.min_height_m,
            ElevatorConstants.max_height_m,
            True,
            0,
            [0.001]
        )

        self.lift_m2d = Mechanism2d(20, ElevatorConstants.max_height_in)
        lift_root = self.lift_m2d.getRoot("Lift Root", 10, 0)
        self.elevator_m2d = lift_root.appendLigament("Elevator",
                                                     self.elevator_sim.getPositionInches(),
                                                     ElevatorConstants.elevator_angle_degrees,
                                                     6,
                                                     Color8Bit(Color.kRed))

    def set_state(self, state: str) -> None:
        self.state = state
        self.lift_main.set_control(self.lift_main_mm.with_position(self.state_values[state]).with_slot(0))

    def get_state(self) -> str:
        return self.state

    def get_position(self) -> float:
        return self.lift_main.get_position(True).value_as_double

    def get_at_target(self) -> bool:
        if (self.state_values[self.state] - ElevatorConstants.elevator_at_target_threshold < self.get_position() <=
                self.state_values[self.state] + ElevatorConstants.elevator_at_target_threshold):
            return True
        else:
            return False

    def set_voltage_direct(self, output: float):
        self.lift_main.set_control(self.lift_main_vo.with_output(output)
                                   .with_limit_forward_motion(self.get_forward_limit_triggered())
                                   .with_limit_reverse_motion(self.get_reverse_limit_triggered()))

    def get_forward_limit_triggered(self) -> bool:
        if self.lift_main.get_position().value_as_double > ElevatorConstants.elevator_upper_limit:
            return True
        else:
            return False

    def get_reverse_limit_triggered(self) -> bool:
        if self.lift_main.get_position().value_as_double < ElevatorConstants.elevator_lower_limit:
            return True
        else:
            return False

    def update_sim(self) -> None:
        current_time = get_current_time_seconds()
        dt = current_time - self.last_time
        self.last_time = current_time

        self.elevator_sim.setInput(0, self.lift_sim.motor_voltage)
        self.elevator_sim.update(dt)
        self.lift_sim.set_raw_rotor_position(self.elevator_sim.getPositionInches() * ElevatorConstants.gearbox_ratio /
                                             (ElevatorConstants.drum_diameter_in * pi))
        self.lift_sim.set_rotor_velocity(self.elevator_sim.getVelocity() * ElevatorConstants.gearbox_ratio)

    def periodic(self) -> None:
        if is_simulation():
            self.update_sim()
            self.elevator_m2d.setLength(self.elevator_sim.getPositionInches())
        else:
            self.elevator_m2d.setLength(self.lift_main.get_position().value_as_double * pi *
                                        ElevatorConstants.drum_diameter_in)

        SmartDashboard.putData("Elevator M2D", self.lift_m2d)
        SmartDashboard.putString("Elevator Position", str(self.elevator_sim.getPositionInches()))
        SmartDashboard.putString("Lift Motor Position", str(self.lift_main.get_position()))
        SmartDashboard.putString("Elevator Setpoint", str(self.lift_main.get_motor_output_status()))
