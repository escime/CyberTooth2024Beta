from commands2 import Subsystem

from phoenix6.hardware import TalonFX
from phoenix6.controls import VoltageOut, MotionMagicVelocityVoltage, Follower
from phoenix6.configs import TalonFXConfiguration
from phoenix6.status_code import StatusCode
from phoenix6.signals import InvertedValue
from phoenix6.utils import get_current_time_seconds, is_simulation

from wpilib import SmartDashboard, DigitalInput
from wpilib.simulation import FlywheelSim
from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations

from math import pi, degrees


class FlywheelSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self._last_sim_time = get_current_time_seconds()
        self.state_values = {"safety": 167}  # In rotations per second
        self.state = "off"
        self.auto_velocity = 0

        self.flywheel = TalonFX(30, "rio")
        self.flywheel_follower = TalonFX(31, "rio")

        self.flywheel_mm = MotionMagicVelocityVoltage(0, enable_foc=False)
        self.flywheel_configs = TalonFXConfiguration()

        self.flywheel_configs.current_limits.stator_current_limit = 40
        self.flywheel_configs.current_limits.stator_current_limit_enable = True
        self.flywheel_gear_ratio = 0.5
        self.flywheel_configs.feedback.sensor_to_mechanism_ratio = self.flywheel_gear_ratio
        self.flywheel_configs.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        self.flywheel_mm_configs = self.flywheel_configs.motion_magic
        self.flywheel_mm_configs.motion_magic_cruise_velocity = 100
        self.flywheel_mm_configs.motion_magic_acceleration = 100
        self.flywheel_mm_configs.motion_magic_jerk = 100

        self.flywheel_slot0_configs = self.flywheel_configs.slot0
        self.flywheel_slot0_configs.with_k_s(0.25)
        self.flywheel_slot0_configs.with_k_v(0.09)
        self.flywheel_slot0_configs.with_k_a(1.56)
        self.flywheel_slot0_configs.with_k_p(1)
        self.flywheel_slot0_configs.with_k_i(0)
        self.flywheel_slot0_configs.with_k_d(0)

        self.gp_sensor = DigitalInput(0)

        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.flywheel.configurator.apply(self.flywheel_configs)
            status = self.flywheel_follower.configurator.apply(self.flywheel_configs)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        self.flywheel_follower.set_control(Follower(30, True))

        self.flywheel_sim = self.flywheel.sim_state

        self.wheel_sim = FlywheelSim(
            DCMotor.krakenX60(2),
            self.flywheel_gear_ratio,
            0.003
        )

        self.flywheel_volts = VoltageOut(0, False)

        self.last_time = get_current_time_seconds()
        self.setpoint_enabled_time = get_current_time_seconds()

    def set_state(self, state: str) -> None:
        self.setpoint_enabled_time = get_current_time_seconds()
        self.state = state
        if state == "auto":
            self.flywheel.set_control(self.flywheel_mm.with_velocity(self.auto_velocity).with_slot(0))
        elif state == "off":
            self.flywheel.set_control(self.flywheel_volts.with_output(0))
        else:
            self.flywheel.set_control(self.flywheel_mm.with_velocity(self.state_values[state]).with_slot(0))

    def set_flywheel_auto_velocity(self, velocity: float) -> None:
        self.auto_velocity = velocity

    def get_state(self) -> str:
        return self.state

    def get_sensor_on(self) -> bool:
        return not self.gp_sensor.get()

    def get_velocity(self) -> float:
        return self.flywheel.get_velocity(True).value_as_double

    def get_at_target(self) -> bool:
        if self.state == "off":
            return True
        elif self.state == "auto":
            if self.auto_velocity - 9 < self.get_velocity() <= self.auto_velocity + 9:
                return True
            else:
                return False
        else:
            if self.state_values[self.state] - 9 < self.get_velocity() <= self.state_values[self.state] + 9:
                return True
            else:
                return False

    def set_voltage_direct(self, output: float):
        self.flywheel.set_control(self.flywheel_volts.with_output(output))

    def update_sim(self):
        current_time = get_current_time_seconds()
        dt = current_time - self.last_time
        self.last_time = current_time
        self.wheel_sim.setInput(0, self.flywheel_sim.motor_voltage)
        self.wheel_sim.update(dt)
        self.flywheel_sim.set_rotor_velocity(radiansToRotations(self.wheel_sim.getAngularVelocity() * self.flywheel_gear_ratio))

    def periodic(self) -> None:
        if is_simulation():
            self.update_sim()

        SmartDashboard.putNumber("Flywheel Velocity", self.get_velocity())
        SmartDashboard.putBoolean("Flywheel at Speed", self.get_at_target())
        SmartDashboard.putNumber("Time Since Setpoint Activated", get_current_time_seconds() - self.setpoint_enabled_time)
