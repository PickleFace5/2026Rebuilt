from enum import auto, Enum

from phoenix6.configs import CANrangeConfiguration, TalonFXConfiguration, MotorOutputConfigs, FeedbackConfigs, ProximityParamsConfigs, CurrentLimitsConfigs
from phoenix6.signals import NeutralModeValue

from pykit.autolog import autologgable_output
from pykit.logger import Logger
from wpilib import Alert
from typing import Callable, Final
from constants import Constants
from subsystems import StateSubsystem
from subsystems.launcher.io import LauncherIO
from wpimath.geometry import Pose2d
from commands2.button import Trigger
from commands2 import InstantCommand

class LauncherSubsystem(StateSubsystem):
    """
    The LauncherSubsystem is responsible for controlling the end effector's compliant wheels.
    """

    class SubsystemState(Enum):
        IDLE = auto()
        SCORE = auto()
        PASS = auto()
        
    _state_configs: dict[SubsystemState, float] = {
        SubsystemState.IDLE: 10.0,
        SubsystemState.SCORE: 28.5,
        SubsystemState.PASS: 20.0,
    }

    def __init__(self, io: LauncherIO, robot_pose_supplier: Callable[[], Pose2d]) -> None:
        super().__init__("Launcher", self.SubsystemState.SCORE)

        self._io: Final[LauncherIO] = io
        self._inputs = LauncherIO.LauncherIOInputs()
        self._robot_pose_supplier = robot_pose_supplier()
        
        # Alert for disconnected motor
        self._motorDisconnectedAlert = Alert("Launcher motor is disconnected.", Alert.AlertType.kError)

        self._primary_trigger = Trigger(lambda: self.find_position() <= 4.667)

        self._primary_trigger.onChange(
            InstantCommand(lambda: self.set_desired_state())
        )


    def periodic(self) -> None:
        """Called periodically to update inputs and log data."""
        # Update inputs from hardware/simulation
        self._io.updateInputs(self._inputs)
        
        # Log inputs to PyKit
        Logger.processInputs("Launcher", self._inputs)
        
        # Update alerts
        self._motorDisconnectedAlert.set(not self._inputs.motorConnected)

    def set_desired_state(self) -> None:
        if not super().set_desired_state(desired_state):
            return
        
        if True: # results change based on whether hub is active, based on fms stuff
            desired_state = self.SubsystemState.SCORE if self.find_position() <= 4.667 else self.SubsystemState.PASS
        else:
            desired_state = self.SubsystemState.IDLE


        # Get motor voltage for this state
        motor_rps = self._state_configs.get(
            desired_state, 
            12.0
        )
        
        # Set motor voltage through IO layer
        self._io.setMotorRPS(motor_rps)

    def find_position(self) -> float:
        return self._robot_pose_supplier.X
