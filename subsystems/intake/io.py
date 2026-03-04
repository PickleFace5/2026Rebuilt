"""IOs for the intake subsystem"""
from dataclasses import dataclass
from math import pi
from typing import Final

from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VelocityVoltage, VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue, InvertedValue
from pykit.autolog import autolog
from wpilib.simulation import DCMotorSim
from wpimath.controller import PIDController
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radians, radians_per_second, amperes, celsius

from constants import Constants
from util import try_until_ok


class IntakeIO:
    """
    Abstract base class for intake IO implementations.
    Provides the interface for both real hardware and simulation.
    """

    @autolog
    @dataclass(slots=True)
    class IntakeIOInputs:
        """Inputs from the intake hardware/simulation."""
        # Motor status
        motor_connected: bool = False
        motor_position: radians = 0.0
        motor_velocity: radians_per_second = 0.0
        motor_applied_volts: float = 0.0
        motor_current: amperes = 0.0
        motor_temperature: celsius = 0.0

    def update_inputs(self, inputs: IntakeIOInputs) -> None:
        """Update the inputs with current hardware/simulation state."""

    def set_motor_rps(self, rps: float) -> None:
        """Set the motor output RPS (revolutions per second)."""


# pylint: disable=too-many-instance-attributes
class IntakeIOTalonFX(IntakeIO):
    """
    Real hardware implementation using TalonFX motor controller.
    """

    def __init__(self) -> None:
        """
        Initialize the real hardware IO.
        """
        self._motor: Final[TalonFX] = TalonFX(
            Constants.CanIDs.INTAKE_TALON,
            "rio"
        )

        # Apply motor configuration
        _motor_config = TalonFXConfiguration()

        _motor_config.slot0 = Constants.IntakeConstants.GAINS
        _motor_config.motor_output.neutral_mode = NeutralModeValue.COAST
        _motor_config.feedback.sensor_to_mechanism_ratio = (
            Constants.IntakeConstants.GEAR_RATIO)
        _motor_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

        try_until_ok(
            5,
            lambda: self._motor.configurator.apply(_motor_config, 0.25)
        )

        # Create status signals for motor
        self._position: Final = self._motor.get_position()
        self._velocity: Final = self._motor.get_velocity()
        self._appliedVolts: Final = self._motor.get_motor_voltage()
        self._current: Final = self._motor.get_stator_current()
        self._temperature: Final = self._motor.get_device_temp()

        # Configure update frequencies
        BaseStatusSignal.set_update_frequency_for_all(
            50,
            self._position,
            self._velocity,
            self._appliedVolts,
            self._current,
            self._temperature
        )
        self._motor.optimize_bus_utilization()

        # Velocity Voltage control request
        self._velocityRequest: Final[VelocityVoltage] = VelocityVoltage(0)

        # Voltage control request
        self._voltageRequest: Final[VoltageOut] = VoltageOut(0)

    def update_inputs(self, inputs: IntakeIO.IntakeIOInputs) -> None:
        """Update inputs with current motor state."""
        # Refresh all motor signals
        motor_status = BaseStatusSignal.refresh_all(
            self._position,
            self._velocity,
            self._appliedVolts,
            self._current,
            self._temperature
        )

        # Update motor inputs
        inputs.motor_connected = motor_status.is_ok()
        inputs.motor_position = self._position.value_as_double
        inputs.motor_velocity = self._velocity.value_as_double
        inputs.motor_applied_volts = self._appliedVolts.value_as_double
        inputs.motor_current = self._current.value_as_double
        inputs.motor_temperature = self._temperature.value_as_double

    def set_motor_rps(self, rps: float) -> None:
        """Set the motor output voltage."""
        if rps == 0:
            self._motor.set_control(self._voltageRequest)
        else:
            self._velocityRequest.velocity = rps
            self._motor.set_control(self._velocityRequest)


class IntakeIOSim(IntakeIO):
    """
    Simulation implementation for testing without hardware.
    """

    def __init__(self) -> None:
        """Initialize the simulation IO."""
        self._motor_type = DCMotor.krakenX60(1)

        linear_system = LinearSystemId.DCMotorSystem(
            self._motor_type,
            Constants.IntakeConstants.MOMENT_OF_INERTIA,
            Constants.IntakeConstants.GEAR_RATIO
        )
        self._sim_motor = DCMotorSim(linear_system, self._motor_type, [0, 0])
        self._closed_loop = True

        self._motor_position: float = 0.0
        self._motor_velocity: float = 0.0
        self._motor_applied_volts: float = 0.0

        self._controller = PIDController(
            Constants.IntakeConstants.GAINS.k_p / (2 * pi),
            Constants.IntakeConstants.GAINS.k_i / (2 * pi),
            Constants.IntakeConstants.GAINS.k_d / (2 * pi),
            0.02
        )

    def update_inputs(self, inputs: IntakeIO.IntakeIOInputs) -> None:
        """Update inputs with simulated state."""

        self._sim_motor.update(0.02)

        if self._closed_loop:
            self._motor_applied_volts = self._controller.calculate(
                self._sim_motor.getAngularVelocity()
            )
        else:
            self._controller.reset()

        self._sim_motor.setInputVoltage(self._motor_applied_volts)

        # Update inputs
        inputs.motor_connected = True
        inputs.motor_velocity = self._sim_motor.getAngularVelocity()
        inputs.motor_applied_volts = self._sim_motor.getInputVoltage()
        inputs.motor_current = self._sim_motor.getCurrentDraw()
        inputs.motor_temperature = 25.0
        inputs.motor_position += inputs.motor_velocity * 0.02

    def set_motor_rps(self, rps: float) -> None:
        """Set the motor output RPS (revolutions per second) (simulated)."""
        self._controller.setSetpoint(rps)
