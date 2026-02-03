import importlib.util
import sys
from pathlib import Path

from enum import auto, Enum
from typing import Final

from pykit.autolog import autologgable_output
from pykit.logger import Logger
from wpilib import Alert

from constants import Constants
from subsystems import StateSubsystem

from subsystems.climber.io import ClimberIO, ClimberIOTalonFX, ClimberIOSim

__all__ = ["ClimberIO", "ClimberIOTalonFX", "ClimberIOSim", "ClimberSubsystem"]

@autologgable_output
class ClimberSubsystem(StateSubsystem):
    """
    The ClimberSubsystem is responsible for controlling the robot's climber mechanism.
    Uses PyKit IO layer for hardware abstraction.
    """

    class SubsystemState(Enum):
        STOW = auto()
        EXTEND = auto()

    _state_configs: dict[SubsystemState, tuple[float]] = {
        SubsystemState.STOW: (0.0),
        SubsystemState.EXTEND: (Constants.ClimberConstants.CLIMB_FULL_THRESHOLD)
    }

    def __init__(self, io: ClimberIO) -> None:
        """
        Initialize the climber subsystem.

        :param io: The climber IO implementation (ClimberIOTalonFX for real hardware, ClimberIOSim for simulation)
        """
        super().__init__("Climber", self.SubsystemState.STOW)
        
        self._io: Final[ClimberIO] = io
        self._inputs = ClimberIO.ClimberIOInputs()
        
        # Alert for disconnected motor
        self._motorDisconnectedAlert = Alert("Climber motor is disconnected.", Alert.AlertType.kError)

    def periodic(self) -> None:
        """Called periodically to update inputs and log data."""
        # Update inputs from hardware/simulation
        self._io.updateInputs(self._inputs)
        
        # Log inputs to PyKit
        Logger.processInputs("Climber", self._inputs)
        
        # Update alerts
        self._motorDisconnectedAlert.set(not self._inputs.motorConnected)

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        """
        Set the desired climber state.

        :param desired_state: The desired state to transition to
        """
        if not super().set_desired_state(desired_state):
            return

        # Get motor voltage for this state
        motor_voltage = self._state_configs.get(
            desired_state, 
            (0.0)
        )
        
        # Set motor voltage through IO layer
        self._io.setMotorVoltage(motor_voltage)

    def get_position(self) -> float:
        return self._io.get_position()