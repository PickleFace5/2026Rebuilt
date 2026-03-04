"""Logic abstraction for Intake IO layers"""
from enum import IntEnum, auto
from typing import Final

from pykit.logger import Logger
from wpilib import Alert

from subsystems import StateSubsystem
from subsystems.intake.io import IntakeIO, IntakeIOSim, IntakeIOTalonFX


class IntakeSubsystem(StateSubsystem):
    """
    The IntakeSubsystem is responsible for controlling the end effector's
    compliant wheels.
    """

    class SubsystemState(IntEnum):
        """Available subsystem states"""
        STOP = auto()
        INTAKE = auto()
        OUTPUT = auto()

    _state_configs: dict[SubsystemState, float] = {
        SubsystemState.STOP: 0.0,
        SubsystemState.INTAKE: 12.5,
        SubsystemState.OUTPUT: -12.5,
    }

    def __init__(self, intake_io: IntakeIO) -> None:
        super().__init__("Intake", self.SubsystemState.STOP)

        self._io: Final[IntakeIO] = intake_io
        self._inputs = IntakeIO.IntakeIOInputs()

        # Alert for disconnected motor
        self._motor_disconnected_alert = Alert(
            "Intake motor is disconnected.",
            Alert.AlertType.kError
        )

    def periodic(self) -> None:
        """Called periodically to update inputs and log data."""
        # Update inputs from hardware/simulation
        self._io.update_inputs(self._inputs)

        # Log inputs to PyKit
        Logger.processInputs("Intake", self._inputs)

        # Update alerts
        self._motor_disconnected_alert.set(not self._inputs.motor_connected)

        super().periodic()

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if not super().set_desired_state(desired_state):
            return

        # Get motor voltage for this state
        motor_rps = self._state_configs.get(
            desired_state,
            0.0
        )

        # Set motor RPS through IO layer
        self._io.set_motor_rps(motor_rps)
