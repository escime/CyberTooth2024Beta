from commands2 import Subsystem

from phoenix6.hardware import TalonFX
from phoenix6.controls import MotionMagicVoltage, VoltageOut, Follower
from phoenix6.configs import TalonFXConfiguration
from phoenix6.status_code import StatusCode
from phoenix6.utils import get_current_time_seconds, is_simulation

from wpilib import Mechanism2d, Color8Bit, Color, SmartDashboard
from wpilib.simulation import ElevatorSim
from wpimath.system.plant import DCMotor
from wpimath.units import inchesToMeters, lbsToKilograms, radiansToRotations

from math import pi, degrees


class ElevatorSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self._last_sim_time = get_current_time_seconds()
        self.state_values = {"stow": 0, "max": 60}  # TODO replace with constants
        self.state = "stow"
        self.last_time = get_current_time_seconds()

        self.lift_main_mm = MotionMagicVoltage(0, enable_foc=False)
        self.lift_main_vo = VoltageOut(0, enable_foc=False)

        use_remote = False
        remote_sensor_id = 31
        for motor_id in [30]:
            motor = TalonFX(motor_id)
            motor_config = TalonFXConfiguration()

            if motor_id == [30][0]:
                if use_remote:
                    motor_config.feedback.feedback_remote_sensor_id = remote_sensor_id
                    motor_config.feedback.feedback_sensor_source = TalonFXConfiguration().feedback.feedback_sensor_source.REMOTE_CANCODER
                else:
                    motor.set_position(0)

            motor_config.current_limits.stator_current_limit = 80  # TODO replace with constants
            motor_config.current_limits.stator_current_limit_enable = True  # TODO replace with constants
            motor_config.feedback.sensor_to_mechanism_ratio = 50  # TODO replace with constants

            motor_mm_config = motor_config.motion_magic
            motor_mm_config.motion_magic_cruise_velocity = 6  # TODO replace with constants
            motor_mm_config.motion_magic_acceleration = 15  # TODO replace with constants
            motor_mm_config.motion_magic_jerk = 100  # TODO replace with constants

            motor_slot0_config = motor_config.slot0
            motor_slot0_config.with_k_g(0.18)  # TODO replace with constants
            motor_slot0_config.with_k_s(0.25)  # TODO replace with constants
            motor_slot0_config.with_k_v(1.59)  # TODO replace with constants
            motor_slot0_config.with_k_a(0.00)  # TODO replace with constants
            motor_slot0_config.with_k_p(60)  # TODO replace with constants
            motor_slot0_config.with_k_i(0)  # TODO replace with constants
            motor_slot0_config.with_k_d(0)  # TODO replace with constants

            status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
            for _ in range(0, 5):
                status = motor.configurator.apply(motor_config)
                if status.is_ok():
                    break
            if not status.is_ok():
                print(f"Could not apply configs, error code: {status.name}")

            if motor_id != [30][0]:
                follower_control_request = Follower([30][0], False)
                motor.set_control(follower_control_request)
            else:
                self.lift_main = motor

        self.lift_sim = self.lift_main.sim_state
        self.elevator_sim = ElevatorSim(
            DCMotor.krakenX60(len([30])),
            50,  # TODO replace with constants
            lbsToKilograms(10),  # TODO replace with constants
            inchesToMeters(2),  # TODO replace with constants
            inchesToMeters(0),  # TODO replace with constants
            inchesToMeters(60),  # TODO replace with constants
            True,
            0,
            [0.01]
        )

        self.lift_m2d = Mechanism2d(20, 50)
        lift_root = self.lift_m2d.getRoot("Lift Root", 10, 0)
        self.elevator_m2d = lift_root.appendLigament("Elevator", self.elevator_sim.getPositionInches(), 90)

    def set_state(self, state: str) -> None:
        self.state = state
        self.lift_main.set_control(self.lift_main_mm.with_position(self.state_values[state]).with_slot(0))

    def get_state(self) -> str:
        return self.state

    def get_position(self) -> float:
        return self.lift_main.get_position(True).value_as_double

    def get_at_target(self) -> bool:
        if self.state_values[self.state] - 0.05 < self.get_position() <= self.state_values[self.state] + 0.05:  # TODO replace with constants
            return True
        else:
            return False

    def set_voltage_direct(self, output: float):
        self.lift_main.set_control(self.lift_main_vo.with_output(output)
                                   .with_limit_forward_motion(self.get_forward_limit_triggered())
                                   .with_limit_reverse_motion(self.get_reverse_limit_triggered()))

    def get_forward_limit_triggered(self) -> bool:
        if self.lift_main.get_position().value_as_double > 60:  # TODO replace with constants
            return True
        else:
            return False

    def get_reverse_limit_triggered(self) -> bool:
        if self.lift_main.get_position().value_as_double < 0:  # TODO replace with constants
            return True
        else:
            return False

    def update_sim(self) -> None:
        current_time = get_current_time_seconds()
        dt = current_time - self.last_time
        self.last_time = current_time

        self.elevator_sim.setInput(0, self.lift_sim.motor_voltage)
        self.elevator_sim.update(dt)
        self.lift_sim.set_raw_rotor_position(radiansToRotations(self.elevator_sim.getPositionInches() * 50))  # TODO replace with constants
        self.lift_sim.set_rotor_velocity(radiansToRotations(self.elevator_sim.getVelocity() * 50))  # TODO replace with constants

    def periodic(self) -> None:
        if is_simulation():
            self.update_sim()
            self.elevator_m2d.setLength(inchesToMeters(self.elevator_sim.getPositionInches()))
        else:
            self.elevator_m2d.setAngle(self.lift_main.get_position().value_as_double)

        SmartDashboard.putData("Elevator M2D", self.lift_m2d)
        SmartDashboard.putString("Elevator Position", str(self.lift_main.get_position()))
