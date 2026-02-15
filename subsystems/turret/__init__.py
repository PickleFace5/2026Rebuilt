from enum import auto, Enum
from pykit.logger import Logger
from wpilib import Alert
from typing import Final, Callable
from constants import Constants
from subsystems import Subsystem
from subsystems.turret.io import TurretIO
from math import *
from wpimath.geometry import Pose2d, Rotation2d
from wpilib import DriverStation
from subsystems import StateSubsystem

class TurretSubsystem(StateSubsystem):

    class SubsystemState(Enum):
        MANUAL = auto()
        HUB = auto()
        DEPOT = auto()
        OUTPOST = auto()

    _state_configs: dict[SubsystemState, bool] = {
        SubsystemState.HUB: True,
        SubsystemState.DEPOT: True,
        SubsystemState.OUTPOST: True,
        SubsystemState.MANUAL: False

    }

    # On the reference there's something about a CANrange here and I don't know what that means so I'm leaving it.
    
    def __init__(self, io: TurretIO, robot_pose_supplier: Callable[[], Pose2d]) -> None:
        super().__init__("Turret", self.SubsystemState.MANUAL)

        self._io: Final[TurretIO] = io
        self._inputs = TurretIO.TurretIOInputs()
        self.set_desired_state(TurretSubsystem.SubsystemState.MANUAL)
        self.robot_pose_supplier = robot_pose_supplier

        self.turret_disconnected_alert = Alert("Turret motor is disconnected.", Alert.AlertType.kError)

        self.independent_rotation = Rotation2d(0)
        self.current_radians = 0.0

    def periodic(self):

        # Update inputs from hardware/simulation
        self._io.update_inputs(self._inputs)

        # Log inputs to PyKit
        Logger.processInputs("Turret", self._inputs)

        # Update alerts
        self.turret_disconnected_alert.set(not self._inputs.turret_connected)

        # TODO will be implemented later for calculating the current position of the turret independent of the robot's rotation
        self.current_radians = self.robot_pose_supplier().rotation().radians() + self.independent_rotation.radians()

        if self.get_current_state() != self.SubsystemState.MANUAL:
            self.rotate_to_goal(self.get_current_state())
        
    def get_radians_to_goal(self):
        # If the robot position is in the alliance side, call get_radians_to_goal before aiming
        # If the robot is in the neutral zone, have it determine what side of the zone it's on so it knows the target to aim at
        ydist = 0.0
        xdist = 0.0
        match self.get_current_state():
            case self.SubsystemState.HUB:
                xdist = abs(self.robot_pose_supplier().X() - Constants.GoalLocations.BLUE_HUB.X()) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(self.robot_pose_supplier().X() - Constants.GoalLocations.RED_HUB.X())
                ydist = abs(self.robot_pose_supplier().Y() - Constants.GoalLocations.BLUE_HUB.Y()) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(self.robot_pose_supplier().Y() - Constants.GoalLocations.RED_HUB.Y())
            case self.SubsystemState.OUTPOST:
                xdist = abs(self.robot_pose_supplier().X() - Constants.GoalLocations.BLUE_OUTPOST_PASS.X()) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(self.robot_pose_supplier().X() - Constants.GoalLocations.RED_OUTPOST_PASS.X())
                ydist = abs(self.robot_pose_supplier().Y() - Constants.GoalLocations.BLUE_OUTPOST_PASS.Y()) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(self.robot_pose_supplier().Y() - Constants.GoalLocations.RED_OUTPOST_PASS.Y())
            case self.SubsystemState.DEPOT:
                xdist = abs(self.robot_pose_supplier().X() - Constants.GoalLocations.BLUE_DEPOT_PASS.X()) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(self.robot_pose_supplier().X() - Constants.GoalLocations.RED_DEPOT_PASS.X())
                ydist = abs(self.robot_pose_supplier().Y() - Constants.GoalLocations.BLUE_DEPOT_PASS.Y()) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(self.robot_pose_supplier().Y() - Constants.GoalLocations.RED_DEPOT_PASS.Y())
            case self.SubsystemState.MANUAL:
                return 0.0
        print(f"Turret: X distance: {xdist}, Y distance: {ydist}")
        if xdist == 0.0:
            return 0.0
        return atan(ydist / xdist)

    def rotate_to_goal(self, target: SubsystemState):
        # This function might not work because it probably isn't periodic so it'll only set the output once and then not check if the angle is correct until it's called again (which is when the target changes)
        
        self.set_desired_state(target)
        if self.get_current_state() != self.SubsystemState.MANUAL:
            target_radians = self.get_radians_to_goal()
            print(f"Turret: Target radians: {target_radians} Subsystem state: {self.get_current_state()}")
            self._io.set_position(target_radians)

    def rotate_manually(self, axis: float): # Axis is the value of the X-axis from a joystick
        target_velocity = axis * Constants.TurretConstants.MAX_MANUAL_VELOCITY
        self._io.set_velocity(target_velocity)

    def get_current_state(self) -> SubsystemState | None:
        """get state"""
        return super().get_current_state()

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        """set state"""
        if not super().set_desired_state(desired_state):
            return

        auto_aim = self._state_configs.get(desired_state, False)

        if auto_aim:
            self.rotate_to_goal(desired_state)
        else:
            self.rotate_manually(0.0)
