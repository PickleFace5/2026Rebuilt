from enum import auto, Enum

from phoenix6.configs import CANrangeConfiguration, TalonFXConfiguration, MotorOutputConfigs, FeedbackConfigs, ProximityParamsConfigs, CurrentLimitsConfigs
from phoenix6.signals import NeutralModeValue

from pykit.autolog import autologgable_output
from pykit.logger import Logger
from wpilib import Alert
from typing import Final
from constants import Constants
from subsystems import StateSubsystem
from subsystems.launcher.io import LauncherIO


class LauncherSubsystem(StateSubsystem):
    """
    The LauncherSubsystem is responsible for controlling the end effector's compliant wheels.
    """

    class SubsystemState(Enum):
        STOP = auto()
        LAUNCH = auto()

    _canrange_config = (CANrangeConfiguration().with_proximity_params(ProximityParamsConfigs().with_proximity_threshold(0.1)))

    _motor_config = (TalonFXConfiguration()
                     .with_slot0(Constants.LauncherConstants.GAINS)
                     .with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.COAST))
                     .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(Constants.LauncherConstants.GEAR_RATIO))
                     .with_current_limits(CurrentLimitsConfigs().with_supply_current_limit_enable(True).with_supply_current_limit(Constants.LauncherConstants.SUPPLY_CURRENT))
                     )


    _state_configs: dict[SubsystemState, tuple[float]] = {
        SubsystemState.STOP: (0.0),
        SubsystemState.LAUNCH: (100.0)
    }

    def __init__(self, io: LauncherIO) -> None:
        super().__init__("Launcher", self.SubsystemState.STOP)

        self._io: Final[LauncherIO] = io
        self._inputs = LauncherIO.LauncherIOInputs()
        
        # Alert for disconnected motor
        self._motorDisconnectedAlert = Alert("Launcher motor is disconnected.", Alert.AlertType.kError)

    def periodic(self) -> None:
        """Called periodically to update inputs and log data."""
        # Update inputs from hardware/simulation
        self._io.updateInputs(self._inputs)
        
        # Log inputs to PyKit
        Logger.processInputs("Launcher", self._inputs)
        
        # Update alerts
        self._motorDisconnectedAlert.set(not self._inputs.motorConnected)

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if not super().set_desired_state(desired_state):
            return

        # Get motor voltage for this state
        motor_rps = self._state_configs.get(
            desired_state, 
            (0.0)
        )
        
        # Set motor voltage through IO layer
        self._io.setMotorRPS(motor_rps)

