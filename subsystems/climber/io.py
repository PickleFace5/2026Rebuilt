from abc import ABC
from dataclasses import dataclass, field
from typing import Final

from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFX
from pykit.autolog import autolog
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radians, radians_per_second, volts, amperes, celsius, rotationsToRadians
from wpimath.controller import ProfiledPIDController, PIDController

from constants import Constants
from util import tryUntilOk


class ClimberIO(ABC):
    """
    Abstract base class for climber IO implementations.
    Provides the interface for both real hardware and simulation.
    """

    @autolog
    @dataclass
    class ClimberIOInputs:
        """Inputs from the climber hardware/simulation."""
        # Motor status
        motorConnected: bool = False
        motorPosition: radians = 0.0
        motorVelocity: radians_per_second = 0.0
        motorAppliedVolts: volts = 0.0
        motorCurrent: amperes = 0.0
        motorTemperature: celsius = 0.0

    def updateInputs(self, inputs: ClimberIOInputs) -> None:
        """Update the inputs with current hardware/simulation state."""
        pass

    def setMotorVoltage(self, voltage: volts) -> None:
        """Set the motor output voltage."""
        pass

class ClimberIOTalonFX(ClimberIO):
    """
    Real hardware implementation using TalonFX motor controller and Servo.
    """

    def __init__(self, motor_id: int, motor_config: TalonFXConfiguration) -> None:
        """
        Initialize the real hardware IO.

        :param motor_id: CAN ID of the TalonFX motor
        :param servo_port: PWM port of the Servo
        :param motor_config: TalonFX configuration to apply
        """
        self._motor: Final[TalonFX] = TalonFX(motor_id, "*")

        # Apply motor configuration
        tryUntilOk(5, lambda: self._motor.configurator.apply(motor_config, 0.25))
        tryUntilOk(5, lambda: self._motor.set_position(0, 0.25))

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

        # Voltage control request
        self._voltage_request: Final[VoltageOut] = VoltageOut(0)

    def updateInputs(self, inputs: ClimberIO.ClimberIOInputs) -> None:
        """Update inputs with current motor and servo state."""
        # Refresh all motor signals
        motorStatus = BaseStatusSignal.refresh_all(
            self._position,
            self._velocity,
            self._appliedVolts,
            self._current,
            self._temperature
        )

        # Update motor inputs
        inputs.motorConnected = motorStatus.is_ok()
        inputs.motorPosition = self._position.value_as_double
        inputs.motorVelocity = self._velocity.value_as_double
        inputs.motorAppliedVolts = self._appliedVolts.value_as_double
        inputs.motorCurrent = self._current.value_as_double
        inputs.motorTemperature = self._temperature.value_as_double

    def setMotorVoltage(self, voltage: volts) -> None:
        """Set the motor output voltage."""
        self._voltage_request.output = voltage
        self._motor.set_control(self._voltage_request)


class ClimberIOSim(ClimberIO):
    """
    Simulation implementation for testing without hardware.
    """

    def __init__(self) -> None:
        """Initialize the simulation IO."""
        self._motor_position: float = 0.0
        self._motor_velocity: float = 0.0
        self._motor_applied_volts: float = 0.0

        self._motor_type = DCMotor.krakenX60FOC(1)
        self._climber_sim = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                self._motor_type, 
                Constants.ClimberConstants.MOMENT_OF_INERTIA, 
                Constants.ClimberConstants.GEAR_RATIO
            ), 
            self._motor_type
        )

        self._closed_loop = False

        self._controller = PIDController(Constants.ClimberConstants.GAINS.k_p,
                                        Constants.ClimberConstants.GAINS.k_i,
                                        Constants.ClimberConstants.GAINS.k_d)


    def updateInputs(self, inputs: ClimberIO.ClimberIOInputs) -> None:
        """Update inputs with simulated state."""
        # Simulate motor behavior (simple integration)
        # In a real simulation, you'd use a physics model here

        if (self._closed_loop):
            self._motor_applied_volts = self._controller.calculate(self._climber_sim.getAngularPosition())
        else:
            self._controller.reset(self._climber_sim.getAngularPosition(), self._climber_sim.getAngularAcceleration())

        self.setMotorVoltage(self._motor_applied_volts)
        self._climber_sim.update(0.02)  # 20ms periodic

        # Update inputs
        inputs.motorConnected = True
        inputs.motorPosition = self._climber_sim.getAngularPosition()
        inputs.motorVelocity = self._climber_sim.getAngularAcceleration()
        inputs.motorAppliedVolts = self._motor_applied_volts
        inputs.motorCurrent = abs(self._climber_sim.getCurrentDraw())  # Rough current estimate
        inputs.motorTemperature = 25.0  # Room temperature

    def setOpenLoop(self, output):
        self._closed_loop = False
        self._motor_applied_volts = output

    def setPosition(self, position):
        self._closed_loop = True
        self._controller.getSetpoint(rotationsToRadians(position))

    def setMotorVoltage(self, voltage: volts) -> None:
        """Set the motor output voltage (simulated)."""
        self._motor_applied_volts = max(-12.0, min(12.0, voltage))
        # Simple velocity model: voltage -> velocity (with some damping)
        self._motor_velocity = self._motor_applied_volts * 10.0  # Adjust multiplier as needed
