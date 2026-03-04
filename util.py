"""Miscellaneous functions used throughout the robot."""
from typing import Callable, Optional

from phoenix6 import StatusCode
from pykit.inputs.loggablepowerdistribution import LoggedPowerDistribution
from wpilib import PowerDistribution, DriverStation
from wpimath.geometry import Pose2d, Translation2d

from constants import Constants

PHASE_BOUNDARIES = [
    (130, "S1", False),
    (105, "S2", True),
    (80,  "S3", False),
    (55,  "S4", True),
    (30,  "E",  None),  # None = endgame, no hub status
]

def get_game_phase() -> tuple[str, int]:
    """Returns the current game phase and remaining time in that phase."""
    if DriverStation.isAutonomous():
        return "Autonomous", int(DriverStation.getMatchTime())
    if not DriverStation.isTeleop():
        return "Disabled", int(DriverStation.getMatchTime())

    match_time = int(DriverStation.getMatchTime())

    for threshold, label, winner_active in PHASE_BOUNDARIES:
        if match_time > threshold:
            status = hub_status(winner_active, DriverStation.getAlliance())
            status_str = "Unknown" if status is None else "Active" if status else "Inactive"
            return f"{label} ({status_str})", match_time - threshold

    return "E (Active)", match_time

def hub_status(winner_active: bool, alliance: DriverStation.Alliance) -> Optional[bool]:
    """
    Returns True if the alliance's hub is active, False otherwise.
    Returns None if there's no game message.
    """
    message = DriverStation.getGameSpecificMessage()
    if not message:
        return None
    return winner_active == ((alliance == DriverStation.Alliance.kBlue) == (message == "B"))

def make_turret_pose_supplier(
    robot_pose_supplier: Callable[[], Pose2d],
) -> Callable[[], Pose2d]:
    """
    Returns a pose supplier that gives the turret center in field frame.
    Use this for hood/launcher/turret distance and aiming so calculations
    use the turret position (robot center + TURRET_OFFSET behind) instead of robot center.
    """
    # Offset in robot frame: turret is TURRET_OFFSET m behind center (robot +X = forward)
    offset_robot = Translation2d(Constants.TURRET_OFFSET, 0.0)

    def get_turret_pose() -> Pose2d:
        robot = robot_pose_supplier()
        offset_field = offset_robot.rotateBy(robot.rotation())
        t = robot.translation()
        turret_translation = Translation2d(t.X() + offset_field.X(), t.Y() + offset_field.Y())
        return Pose2d(turret_translation, robot.rotation())

    return get_turret_pose

def try_until_ok(max_attempts: int, command: Callable[[], StatusCode]) -> None:
    """Attempts to run the command until no error is produced."""
    for _ in range(max_attempts):
        error = command()
        if error.is_ok():
            break

def install_safe_power_distribution_logging() -> None:
    """
    Install a Power Distribution logger that does not spam CAN errors when no PDH/PDP
    is present. PyKit's Logger calls LoggedPowerDistribution.getInstance().saveToTable()
    every cycle; if no device exists at the expected CAN ID, that causes repeated
    "CAN: Message not found: Module N" errors.
    """
    module_id = getattr(
        Constants.CanIDs, "POWER_DISTRIBUTION_MODULE_ID", None
    )
    if module_id is not None:
        # PDH/PDP present: use real device but catch CAN errors so one bad cycle doesn't spam
        class _SafeLoggedPowerDistribution(LoggedPowerDistribution):
            def saveToTable(self, table):
                try:
                    super().saveToTable(table)
                except Exception:
                    table.put("Voltage", 0.0)
                    table.put("TotalCurrent", 0.0)
                    table.put("TotalPower", 0.0)
                    table.put("TotalEnergy", 0.0)
                    table.put("Temperature", 0.0)
                    table.put("ChannelCurrentsList", [])
                    table.put("ChannelCurrentsTotal", 0.0)

        LoggedPowerDistribution.instance = _SafeLoggedPowerDistribution(
            moduleId=module_id,
            moduleType=PowerDistribution.ModuleType.kRev,
        )
    else:
        # No PDH: stub that never touches CAN
        class _StubLoggedPowerDistribution(LoggedPowerDistribution):
            def __init__(self) -> None:
                self.moduleId = 0
                self.moduleType = PowerDistribution.ModuleType.kRev
                self.distribution = None

            def saveToTable(self, table):
                table.put("Voltage", 0.0)
                table.put("TotalCurrent", 0.0)
                table.put("TotalPower", 0.0)
                table.put("TotalEnergy", 0.0)
                table.put("Temperature", 0.0)
                table.put("ChannelCurrentsList", [])
                table.put("ChannelCurrentsTotal", 0.0)

        LoggedPowerDistribution.instance = _StubLoggedPowerDistribution()
