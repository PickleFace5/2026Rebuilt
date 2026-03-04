"""Logic abstraction for Launcher IO layers"""
from enum import IntEnum, auto
from math import pi
from typing import Callable, Final, Optional

from pathplannerlib.auto import FlippingUtil, AutoBuilder
from pykit.logger import Logger
from wpilib import Alert
from wpimath.geometry import Pose2d

from constants import Constants
from subsystems import StateSubsystem
from subsystems.launcher.io import LauncherIO, LauncherIOTalonFX, LauncherIOSim

LauncherConstants = Constants.LauncherConstants
GeneralConstants = Constants.GeneralConstants


def velocity_to_wheel_rps(velocity: float) -> float:
    """Converts m/s to rotations per second for the flywheel, accounting for
    inertia."""

    effective_rotational_inertia = 7 * GeneralConstants.GAME_PIECE_WEIGHT * (
            LauncherConstants.FLYWHEEL_RADIUS ** 2)

    speed_transfer_percentage = (
            (20 * LauncherConstants.MOMENT_OF_INERTIA)
            /
            (effective_rotational_inertia + (
                    40 * LauncherConstants.MOMENT_OF_INERTIA))
    )

    rpm = velocity / (
            LauncherConstants.FLYWHEEL_RADIUS * speed_transfer_percentage)

    return rpm / (2 * pi)


# pylint: disable=too-many-instance-attributes
class LauncherSubsystem(StateSubsystem):
    """
    The LauncherSubsystem is responsible for controlling the end effector's
    compliant wheels.
    """

    class SubsystemState(IntEnum):
        """Available subsystem states"""
        IDLE = auto()
        SCORE = auto()
        PASS = auto()

    _state_configs: dict[SubsystemState, float] = {
        # Meters per second
        SubsystemState.IDLE: 28.0,  # velocityToWheelRPS(5.0),
        SubsystemState.SCORE: 30.0,  # velocityToWheelRPS(12.26),
        SubsystemState.PASS: 50.0  # velocityToWheelRPS(10.0),
    }

    def __init__(self,
                 launcher_io: LauncherIO,
                 robot_pose_supplier: Callable[[], Pose2d]
                 ) -> None:
        super().__init__("Launcher", self.SubsystemState.SCORE)

        self._io: Final[LauncherIO] = launcher_io
        self.inputs = LauncherIO.LauncherIOInputs()
        self._robot_pose_supplier = robot_pose_supplier
        self._desired_projectile_velocity = 0.0
        self.desired_motor_rps = 0.0
        self.distance = 1.0
        self._aiming_rps: Optional[
            float] = None  # From unified aiming LUT (RPS)

        self._motor_disconnected_alert = Alert(
            "Launcher motor is disconnected.",
            Alert.AlertType.kError
        )

        self.set_desired_state(self.SubsystemState.IDLE)

    def set_aiming_setpoint(self, rps: Optional[float]) -> None:
        """Set launcher RPS from unified aiming LUT."""
        self._aiming_rps = rps

    def periodic(self) -> None:
        """Called periodically to update inputs and log data."""
        # Update inputs from hardware/simulation
        self._io.update_inputs(self.inputs)

        # When in SCORE, use unified aiming LUT RPS (fallback to base config
        # if not set)
        if self.get_current_state() == self.SubsystemState.SCORE:
            hub_pose = (
                Constants.FieldConstants.HUB_POSE
                if not AutoBuilder.shouldFlip()
                else FlippingUtil.flipFieldPose(
                    Constants.FieldConstants.HUB_POSE
                )
            )
            self.distance = (
                self._robot_pose_supplier()
                .translation()
                .distance(hub_pose.translation())
            )
            self.desired_motor_rps = (
                self._aiming_rps
                if self._aiming_rps is not None
                else self._state_configs[self.SubsystemState.SCORE]
            )
            self._io.set_motor_rps(self.desired_motor_rps)

        # Log inputs to PyKit
        Logger.processInputs("Launcher", self.inputs)

        # Log outputs to PyKit
        Logger.recordOutput(
            "Launcher/Target Projectile Velocity",
            self._desired_projectile_velocity
        )
        Logger.recordOutput(
            "Launcher/Target Motor RPS",
            self.desired_motor_rps
        )
        Logger.recordOutput("Launcher/DistanceToHub", self.distance)

        # Update alerts
        self._motor_disconnected_alert.set(not self.inputs.motor_connected)

        super().periodic()

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if not super().set_desired_state(desired_state):
            return

        projectile_velocity = self._state_configs.get(
            desired_state,
            0.0
        )  # self.get_aim_velocity(desired_state)

        self.desired_motor_rps = projectile_velocity
        self._io.set_motor_rps(self.desired_motor_rps)
