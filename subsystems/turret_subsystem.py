import phoenix6.utils
from commands2 import Subsystem

from phoenix6.hardware import TalonFX
from phoenix6.controls import MotionMagicVoltage, VoltageOut
from phoenix6.configs import TalonFXConfiguration
from phoenix6.status_code import StatusCode
from phoenix6.signals import InvertedValue
from phoenix6.utils import get_current_time_seconds, is_simulation

from rev import CANSparkMax

from wpilib import Mechanism2d, Color8Bit, Color, SmartDashboard, DigitalInput
from wpilib.simulation import SingleJointedArmSim
from wpimath.system.plant import DCMotor
from wpimath.units import inchesToMeters, lbsToKilograms, radiansToRotations
from wpimath.geometry import Pose2d

from math import pi, degrees, atan2


class TurretSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self._last_sim_time = get_current_time_seconds()
        self.state_values = {"zero": 0, "reverse": 1}
        self.state = "zero"
        self.auto_position = 0
        self.limit_around_zero = 0.25

        self.turret = TalonFX(30, "rio")
        self.turret.set_position(0)

        self.turret_mm = MotionMagicVoltage(0, enable_foc=False)
        self.turret_configs = TalonFXConfiguration()

        self.turret_configs.current_limits.stator_current_limit = 20
        self.turret_configs.current_limits.stator_current_limit_enable = True
        self.turret_gear_ratio = 16
        self.turret_configs.feedback.sensor_to_mechanism_ratio = self.turret_gear_ratio
        if not phoenix6.utils.is_simulation():
            self.turret_configs.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        else:
            self.turret_configs.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        self.turret_mm_configs = self.turret_configs.motion_magic
        self.turret_mm_configs.motion_magic_cruise_velocity = 1
        self.turret_mm_configs.motion_magic_acceleration = 10
        self.turret_mm_configs.motion_magic_jerk = 100

        self.turret_slot0_configs = self.turret_configs.slot0
        self.turret_slot0_configs.with_k_s(0.25)
        self.turret_slot0_configs.with_k_v(0.4)
        self.turret_slot0_configs.with_k_a(0.02)
        self.turret_slot0_configs.with_k_p(90)
        self.turret_slot0_configs.with_k_i(0)
        self.turret_slot0_configs.with_k_d(0)

        self.gp_sensor = DigitalInput(0)

        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.turret.configurator.apply(self.turret_configs)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        self.turret_m2d = Mechanism2d(6, 6)
        self.turret_m2d_root = self.turret_m2d.getRoot("turret", 3, 3)
        self.turret_m2d_ligament = self.turret_m2d_root.appendLigament("ligament", 1.5, 0, 6,
                                                                       Color8Bit(Color.kRed))

        self.turret_motor_sim = self.turret.sim_state
        self.turret_sim = SingleJointedArmSim(
            DCMotor.krakenX60(1),
            self.turret_gear_ratio,
            SingleJointedArmSim.estimateMOI(inchesToMeters(1), lbsToKilograms(1)),
            inchesToMeters(20),
            -2 * pi,
            2 * pi,
            False,
            1
        )

        self.turret_volts = VoltageOut(0, False)

        self.last_time = get_current_time_seconds()

    def set_state(self, state: str) -> None:
        self.state = state
        if state == "auto":
            self.turret.set_control(self.turret_mm.with_position(self.auto_position).with_slot(0))
        elif state == "off":
            self.turret.set_control(self.turret_volts.with_output(0))
        else:
            self.turret.set_control(self.turret_mm.with_position(self.state_values[state]).with_slot(0))

    def set_turret_auto_position(self, position: float) -> None:
        self.auto_position = position

    def get_state(self) -> str:
        return self.state

    def get_target_heading(self, robot_pose: Pose2d, target: [float, float]) -> float:
        goal_heading = atan2(target[1] - robot_pose.y, target[0] - robot_pose.x) * 180 / pi
        target_heading = (goal_heading - robot_pose.rotation().degrees()) / 360
        if target_heading < -self.limit_around_zero:
            target_heading = 1 + target_heading
        elif target_heading > 1 + self.limit_around_zero:
            target_heading = target_heading - 1
        return target_heading

    def get_sensor_on(self) -> bool:
        return not self.gp_sensor.get()

    def get_position(self) -> float:
        return self.turret.get_position(True).value_as_double

    def get_at_target(self) -> bool:
        if self.state == "off":
            return True
        elif self.state == "auto":
            if self.auto_position - 0.05 < self.get_position() <= self.auto_position + 0.05:
                return True
            else:
                return False
        else:
            if self.state_values[self.state] - 0.05 < self.get_position() <= self.state_values[self.state] + 0.05:
                return True
            else:
                return False

    def set_voltage_direct(self, output: float):
        self.turret.set_control(self.turret_volts.with_output(output)
                                .with_limit_forward_motion(self.get_forward_limit_triggered())
                                .with_limit_reverse_motion(self.get_reverse_limit_triggered()))

    def get_forward_limit_triggered(self) -> bool:
        if self.turret.get_position().value_as_double > 1 + self.limit_around_zero:
            return True
        else:
            return False

    def get_reverse_limit_triggered(self) -> bool:
        if self.turret.get_position().value_as_double < -self.limit_around_zero:
            return True
        else:
            return False

    def update_sim(self):
        current_time = get_current_time_seconds()
        dt = current_time - self.last_time
        self.last_time = current_time
        self.turret_sim.setInput(0, self.turret_motor_sim.motor_voltage)
        self.turret_sim.update(dt)
        self.turret_motor_sim.set_raw_rotor_position(radiansToRotations(self.turret_sim.getAngle() * self.turret_gear_ratio))
        self.turret_motor_sim.set_rotor_velocity(radiansToRotations(self.turret_sim.getVelocity() * self.turret_gear_ratio))

    def periodic(self) -> None:
        if is_simulation():
            self.update_sim()
            self.turret_m2d_ligament.setAngle(degrees(self.turret_sim.getAngle()))
        else:
            self.turret_m2d_ligament.setAngle(self.turret.get_position().value_as_double)

        if self.state == "auto":
            self.turret.set_control(self.turret_mm.with_position(self.auto_position).with_slot(0))

        SmartDashboard.putData("Turret M2D", self.turret_m2d)
        SmartDashboard.putNumber("Turret Position", self.turret.get_position().value_as_double)
