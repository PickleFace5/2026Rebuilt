from enum import auto, Enum
from typing import Optional

from commands2 import Command, Subsystem, cmd
from wpilib import DriverStation

from subsystems.intake import IntakeSubsystem
from subsystems.feeder import FeederSubsystem
from subsystems.launcher import LauncherSubsystem
from subsystems.hood import HoodSubsystem
from subsystems.turret import TurretSubsystem

from pykit.logger import Logger

from constants import Constants


class Superstructure(Subsystem):
    """
    The Superstructure is in charge of handling all subsystems to ensure no conflicts between them.
    """

    class Goal(Enum):
        DEFAULT     = auto()  # Default goal
        INTAKE      = auto()  # Intaking fuel from the floor.  This goal may be removed
        LAUNCH      = auto()  # Scoring fuel into the hub
        AIMHUB      = auto()  # Point turret to hub
        AIMOUTPOST  = auto()  # Point turret to the outpost side of the alliance center
        AIMDEPOT    = auto()  # Point turret to the depot side of the alliance center

    # Map each goal to each subsystem state to reduce code complexity
    _goal_to_states: dict[Goal,
        tuple[
            Optional[IntakeSubsystem.SubsystemState],
            Optional[FeederSubsystem.SubsystemState],
            Optional[LauncherSubsystem.SubsystemState],
            Optional[HoodSubsystem.SubsystemState],
            Optional[TurretSubsystem.SubsystemState],
            bool, # Superstructure state? (Is it handled by periodic or just a single action?)
        ]] = {

        Goal.DEFAULT : (
            IntakeSubsystem.SubsystemState.STOP,
            FeederSubsystem.SubsystemState.STOP,
            LauncherSubsystem.SubsystemState.IDLE,
            HoodSubsystem.SubsystemState.STOW,
            TurretSubsystem.SubsystemState.HUB,
            True
        ),

        Goal.INTAKE : (
            IntakeSubsystem.SubsystemState.INTAKE,
            FeederSubsystem.SubsystemState.STOP,
            LauncherSubsystem.SubsystemState.IDLE,
            HoodSubsystem.SubsystemState.STOW,
            None, True
        ),

        Goal.LAUNCH : (
            IntakeSubsystem.SubsystemState.INTAKE,
            FeederSubsystem.SubsystemState.INWARD,
            LauncherSubsystem.SubsystemState.SCORE,
            None, None, True
        ),

        Goal.AIMHUB : (
            None, None, None,
            HoodSubsystem.SubsystemState.AIMBOT,
            TurretSubsystem.SubsystemState.HUB,
            False
        ),

        Goal.AIMOUTPOST : (
            None, None, None,
            HoodSubsystem.SubsystemState.PASS,
            TurretSubsystem.SubsystemState.OUTPOST,
            False
        ),

        Goal.AIMDEPOT : (
            None, None, None,
            HoodSubsystem.SubsystemState.PASS,
            TurretSubsystem.SubsystemState.DEPOT,
            False
        ),

    }

    def __init__(self,
            intake: Optional[IntakeSubsystem] = None,
            feeder: Optional[FeederSubsystem] = None,
            launcher: Optional[LauncherSubsystem] = None,
            hood: Optional[HoodSubsystem] = None,
            turret: Optional[TurretSubsystem] = None
                ) -> None:
        """
        Constructs the superstructure using instance of each subsystem.
        Subsystems are optional to support robots that don't have all hardware.

        :param intake: Subsystem that handles the intake
          :type intake: Optional[IntakeSubsystem]
        :param feeder: Subsystem that handles the feeder
          :type feeder: Optional[FeederSubsystem]
        :param launcher: Subsystem that handles the launcher
          :type launcher: Optional[LauncherSubsystem]
        :param hood: Subsystem that handles the hood
          :type hood: Optional[HoodSubsystem]
        :param turret: Subsystem that handles the turret
          :type turret: Optional[TurretSubsystem]
        """
        super().__init__()

        self.intake = intake
        self.feeder = feeder
        self.launcher = launcher
        self.hood = hood
        self.turret = turret

        self._goal_state = self.Goal.DEFAULT
        self.set_goal_command(self._goal_state)

        self._turret_check = False
        self._hood_check = False
        self._flywheel_check = False

        self._checks_override = False

    def periodic(self):
        if DriverStation.isDisabled():
            return

        self._turret_check = abs(self.turret.inputs.turret_setpoint - self.turret.inputs.turret_position) < Constants.TurretConstants.SETPOINT_TOLERANCE
        self._hood_check = abs(self.hood.inputs.hood_setpoint - self.hood.inputs.hood_position) < Constants.HoodConstants.SETPOINT_TOLERANCE
        self._flywheel_check = abs(self.launcher.desired_motorRPS - self.launcher.inputs.motorVelocity) < Constants.LauncherConstants.SETPOINT_TOLERANCE

        match self._goal_state:
            case self.Goal.DEFAULT:
                if self.feeder.is_locked:
                    self.feeder.unlock()
                    self.feeder.set_desired_state(FeederSubsystem.SubsystemState.STOP)

            case self.Goal.INTAKE:
                if self.feeder.is_locked:
                    self.feeder.unlock()
                    self.feeder.set_desired_state(FeederSubsystem.SubsystemState.INWARD)

            case self.Goal.LAUNCH:
                if (self._turret_check and self._hood_check and self._flywheel_check) or self._checks_override:
                    self.feeder.unlock()
                    self.feeder.set_desired_state(FeederSubsystem.SubsystemState.INWARD)

                else:
                    self.feeder.set_desired_state(FeederSubsystem.SubsystemState.STOP)
                    self.feeder.lock()

        Logger.recordOutput("Superstructure/Goal State", self._goal_state.name)
        Logger.recordOutput("Superstructure/Turret Check", self._turret_check)
        Logger.recordOutput("Superstructure/Hood Check", self._hood_check)
        Logger.recordOutput("Superstructure/Flywheel Check", self._flywheel_check)
        Logger.recordOutput("Superstructure/Overridden", self._checks_override)


    def _set_goal(self, goal: Goal) -> None:

        intake_state, feeder_state, launcher_state, hood_state, turret_state, superstructure_state = self._goal_to_states.get(goal, (None, None, None, None, None, False))

        if not intake_state is None:
            self.intake.set_desired_state(intake_state)

        if not feeder_state is None:
            self.feeder.set_desired_state(feeder_state)

        if not launcher_state is None:
            self.launcher.set_desired_state(launcher_state)

        if not hood_state is None:
            self.hood.set_desired_state(hood_state)

        if not turret_state is None:
            self.turret.set_desired_state(turret_state)

        if superstructure_state:
            self._goal_state = goal

    def _set_override(self) -> None:
        self._checks_override = not self._checks_override

    def set_goal_command(self, goal: Goal) -> Command:
        """
        Return a command that sets the superstructure goal to whatever the desired goal is.

        :param goal: The desired goal
        :type goal:  Goal
        :return:     A command that will set the desired goal
        :rtype:      Command
        """
        return cmd.runOnce(lambda: self._set_goal(goal), self)

    def override_checks(self) -> Command:
        return cmd.runOnce(lambda: self._set_override(), self)
