from enum import auto, Enum

from pykit.autolog import autologgable_output
from pykit.logger import Logger
from wpilib import Alert
from typing import Callable, Final
from subsystems import StateSubsystem
from subsystems.launcher.io import LauncherIO, LauncherIOTalonFX, LauncherIOSim
from wpimath.geometry import Pose2d
from commands2.button import Trigger
from commands2 import InstantCommand
from math import pi

from constants import Constants
LauncherConstants = Constants.LauncherConstants
GeneralConstants = Constants.GeneralConstants

def velocityToWheelRPS(velocity: float) -> float:
    """Converts m/s to rotations per second for the flywheel, accounting for inertia."""
        
    effective_rotational_inertia = 7 * GeneralConstants.GAME_PIECE_WEIGHT * (LauncherConstants.FLYWHEEL_RADIUS ** 2)

    speed_transfer_percentage = (
        (20 * LauncherConstants.MOMENT_OF_INERTIA)
        /
        (effective_rotational_inertia + (40 * LauncherConstants.MOMENT_OF_INERTIA))
    )

    rpm = (velocity) / (LauncherConstants.FLYWHEEL_RADIUS * speed_transfer_percentage)
        
    return rpm/(2*pi)

class LauncherSubsystem(StateSubsystem):
    """
    The LauncherSubsystem is responsible for controlling the end effector's compliant wheels.
    """

    class SubsystemState(Enum):
        IDLE = auto()
        SCORE = auto()
        PASS = auto()
        
    _state_configs: dict[SubsystemState, float] = {
        # Meters per second
        SubsystemState.IDLE: 0, #velocityToWheelRPS(5.0),
        SubsystemState.SCORE: velocityToWheelRPS(12.26),
        SubsystemState.PASS: velocityToWheelRPS(10.0),
    }

    def __init__(self, io: LauncherIO, robot_pose_supplier: Callable[[], Pose2d]) -> None:
        super().__init__("Launcher", self.SubsystemState.SCORE)

        self._io: Final[LauncherIO] = io
        self._inputs = LauncherIO.LauncherIOInputs()
        self._robot_pose_supplier = robot_pose_supplier
        self._desired_projectile_velocity = 0.0
        self._desired_motorRPS = 0.0
        
        self._motorDisconnectedAlert = Alert("Launcher motor is disconnected.", Alert.AlertType.kError)

        self.set_desired_state(self.SubsystemState.SCORE)

        """"
        automatic state switching based on position

        self._primary_trigger = Trigger(lambda: self.find_position() <= 4.667)

        self._primary_trigger.onChange(
            InstantCommand(lambda: self.set_desired_state(self.SubsystemState.SCORE if self.find_position() <= 4.667 else self.SubsystemState.PASS))
        )
        """


    def periodic(self) -> None:
        """Called periodically to update inputs and log data."""
        # Update inputs from hardware/simulation
        self._io.updateInputs(self._inputs)
        
        # Log inputs to PyKit
        Logger.processInputs("Launcher", self._inputs)

        # Log outputs to PyKit
        Logger.recordOutput("Launcher/Target Projectile Velocity", self._desired_projectile_velocity)
        Logger.recordOutput("Launcher/Target Motor RPS", self._desired_motorRPS)

        # Update alerts
        self._motorDisconnectedAlert.set(not self._inputs.motorConnected)

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if not super().set_desired_state(desired_state):
            return

        projectile_velocity = self._state_configs.get(
            desired_state, 
            0.0
        )

        self._desired_projectile_velocity = projectile_velocity
        self._desired_motorRPS = velocityToWheelRPS(projectile_velocity)
        self._desired_motorRPS = max(min(self._desired_motorRPS, 75.0), -75)  # Ensure non-negative RPS
        self._io.setMotorRPS(self._desired_motorRPS)

    def find_position(self) -> float:
        return self._robot_pose_supplier().X

