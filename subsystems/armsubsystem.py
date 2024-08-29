from commands2 import Subsystem

from phoenix6.hardware import TalonFX
from phoenix6.controls import MotionMagicVoltage, VoltageOut
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.talon_fx_configs import FeedbackSensorSourceValue
from phoenix6.status_code import StatusCode
from phoenix6.utils import get_current_time_seconds, is_simulation

from rev import CANSparkMax

from wpilib import Mechanism2d, Color8Bit, Color, SmartDashboard, RobotController
from wpilib.simulation import SingleJointedArmSim, DCMotorSim
from wpimath.system.plant import DCMotor
from wpimath.units import inchesToMeters, lbsToKilograms, radiansToRotations

from math import pi, degrees


class ArmSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self._last_sim_time = get_current_time_seconds()
        self.state_values = {"stow": 0, "intake": 0.5, "shoot": 0.4, "reverse_shoot": 0.2}
        self.state = "stow"

        self.elbow = TalonFX(30, "rio")
        self.elbow.set_position(0)

        self.elbow_mm = MotionMagicVoltage(0, enable_foc=False)
        self.elbow_configs = TalonFXConfiguration()

        self.elbow_gear_ratio = 83.2
        self.elbow_configs.feedback.sensor_to_mechanism_ratio = self.elbow_gear_ratio

        self.elbow_mm_configs = self.elbow_configs.motion_magic
        self.elbow_mm_configs.motion_magic_cruise_velocity = 6
        self.elbow_mm_configs.motion_magic_acceleration = 15
        self.elbow_mm_configs.motion_magic_jerk = 100

        self.elbow_slot0_configs = self.elbow_configs.slot0
        self.elbow_slot0_configs.with_k_g(0.18)
        self.elbow_slot0_configs.with_k_s(0.25)
        self.elbow_slot0_configs.with_k_v(1.59)
        self.elbow_slot0_configs.with_k_a(0.00)
        self.elbow_slot0_configs.with_k_p(60)
        self.elbow_slot0_configs.with_k_i(0)
        self.elbow_slot0_configs.with_k_d(0)
        
        self.intake = CANSparkMax(31, CANSparkMax.MotorType.kBrushless)
        self.intake.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.intake.set(0)

        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.elbow.configurator.apply(self.elbow_configs)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        self.arm_m2d = Mechanism2d(3, 2)
        self.arm_m2d_root = self.arm_m2d.getRoot("arm", 1.5, 0.25)
        self.arm_m2d_elbow = self.arm_m2d_root.appendLigament("elbow", 1.5, 0, 6,
                                                              Color8Bit(Color.kRed))

        self.elbow_sim = self.elbow.sim_state
        self.arm_sim = SingleJointedArmSim(
            DCMotor.krakenX60(1),
            self.elbow_gear_ratio,
            SingleJointedArmSim.estimateMOI(inchesToMeters(20), lbsToKilograms(10)),
            inchesToMeters(20),
            -0.1,
            pi + 0.1,
            True,
            1
        )

        self.elbow_volts = VoltageOut(0, False)

        self.last_time = get_current_time_seconds()

    def set_state(self, state: str) -> None:
        self.state = state
        self.elbow.set_control(self.elbow_mm.with_position(self.state_values[state]).with_slot(0))

    def get_state(self) -> str:
        return self.state

    def get_position(self) -> float:
        return self.elbow.get_position(True).value_as_double

    def get_at_target(self) -> bool:
        if self.state_values[self.state] - 0.05 < self.get_position() <= self.state_values[self.state] + 0.05:
            return True
        else:
            return False

    def set_voltage_direct(self, output: float):
        self.elbow.set_control(self.elbow_volts.with_output(output)
                               .with_limit_forward_motion(self.get_forward_limit_triggered())
                               .with_limit_reverse_motion(self.get_reverse_limit_triggered()))

    def get_forward_limit_triggered(self) -> bool:
        if self.elbow.get_position().value_as_double > 0.5:
            return True
        else:
            return False

    def get_reverse_limit_triggered(self) -> bool:
        if self.elbow.get_position().value_as_double < 0:
            return True
        else:
            return False

    def update_sim(self):
        current_time = get_current_time_seconds()
        dt = current_time - self.last_time
        self.last_time = current_time
        self.arm_sim.setInput(0, self.elbow_sim.motor_voltage)
        self.arm_sim.update(dt)
        self.elbow_sim.set_raw_rotor_position(radiansToRotations(self.arm_sim.getAngle() * self.elbow_gear_ratio))
        self.elbow_sim.set_rotor_velocity(radiansToRotations(self.arm_sim.getVelocity() * self.elbow_gear_ratio))

    def periodic(self) -> None:
        if is_simulation():
            self.update_sim()
            self.arm_m2d_elbow.setAngle(degrees(self.arm_sim.getAngle()))
        else:
            self.arm_m2d_elbow.setAngle(self.elbow.get_position().value_as_double)

        if self.state == "shoot" or self.state == "reverse_shoot":
            if self.get_at_target():
                self.intake.setVoltage(-12)
        elif self.state == "intake":
            self.intake.setVoltage(10)
        else:
            self.intake.setVoltage(1)

        SmartDashboard.putData("Arm M2D", self.arm_m2d)
        SmartDashboard.putString("Arm Simulation Position", str(self.arm_sim.getAngle()))
        SmartDashboard.putString("Elbow Position", str(self.elbow.get_position()))
        SmartDashboard.putString("Elbow Output Applied", str(self.elbow.get_motor_output_status()))
        SmartDashboard.putBoolean("Elbow at position?", self.get_at_target())