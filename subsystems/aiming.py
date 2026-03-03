"""
Shooting on the Move (SOTM) and Unified Kinematic Aiming.

Provides ShooterAimingTable (distance -> RPM/hood LUT) and get_aiming_parameters()
for Virtual Goal logic so turret, hood, and launcher can aim and shoot while moving.
"""
import math
from dataclasses import dataclass
from typing import Callable

from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds


def _linear_interp(x: float, xs: list[float], ys: list[float]) -> float:
    """Linear interpolation. Clamps to [min(ys), max(ys)] if x outside [min(xs), max(xs)]."""
    if not xs or not ys or len(xs) != len(ys):
        return ys[0] if ys else 0.0
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(len(xs) - 1):
        if xs[i] <= x <= xs[i + 1]:
            t = (x - xs[i]) / (xs[i + 1] - xs[i]) if xs[i + 1] != xs[i] else 0.0
            return ys[i] + t * (ys[i + 1] - ys[i])
    return ys[-1]


class ShooterAimingTable:
    """
    Distance-based LUT for RPM and hood angle (continuous interpolation).
    Replaces zone-based logic to avoid jumps. Populate with golden samples from stationary tuning.
    """
    def __init__(self) -> None:
        # (distance_m, value) sorted by distance. Hood in rotations (motor units).
        self._rpm_dist: list[float] = []
        self._rpm_val: list[float] = []
        self._hood_dist: list[float] = []
        self._hood_val: list[float] = []
        self._seed_defaults()

    def _seed_defaults(self) -> None:
        """Seed with tuned values from stationary testing (distance m, hood rotations, flywheel RPS)."""
        # From testing: Distance (m), Hood (rotations), Flywheel (RPS)
        distance = [1.7, 2.56, 3.5, 4.49, 5.365]
        hood_rotations = [0.0, 0.0048828125, 0.0068359375, 0.02026367188, 0.03100585938]  # from zero (hood down = 0)
        flywheel_rps = [28.0, 30.0, 35.0, 38.0, 44.0]
        self._rpm_dist = list(distance)
        self._rpm_val = list(flywheel_rps)
        self._hood_dist = list(distance)
        self._hood_val = list(hood_rotations)

    def put_rpm(self, distance_m: float, rpm: float) -> None:
        """Add or update one RPM sample (distance in meters)."""
        self._add_sample(distance_m, rpm, self._rpm_dist, self._rpm_val)

    def put_hood(self, distance_m: float, hood_rotations: float) -> None:
        """Add or update one hood angle sample (distance in meters, hood in rotations)."""
        self._add_sample(distance_m, hood_rotations, self._hood_dist, self._hood_val)

    def _add_sample(self, x: float, y: float, xs: list[float], ys: list[float]) -> None:
        if not xs or x < xs[0]:
            xs.insert(0, x)
            ys.insert(0, y)
            return
        if x > xs[-1]:
            xs.append(x)
            ys.append(y)
            return
        for i, xi in enumerate(xs):
            if abs(xi - x) < 1e-9:
                ys[i] = y
                return
            if xi > x:
                xs.insert(i, x)
                ys.insert(i, y)
                return

    def get_settings(self, distance_m: float) -> dict[str, float]:
        """Returns {'rpm': RPS (wheel), 'hood': hood angle in rotations} for the given distance."""
        return {
            "rpm": _linear_interp(distance_m, self._rpm_dist, self._rpm_val),
            "hood": _linear_interp(distance_m, self._hood_dist, self._hood_val),
        }


@dataclass
class AimingParameters:
    """Result of Virtual Goal aiming: setpoints for turret, hood, and launcher."""
    turret_angle_rad: float
    virtual_dist_m: float
    rps: float  # launcher wheel RPS (from LUT, same units as LauncherSubsystem.desired_motorRPS)
    hood_rotations: float  # hood angle in rotations (motor units)


# Default nominal exit velocity (m/s) for time-of-flight. Tune from testing.
NOMINAL_BALL_VELOCITY_MPS = 12.0


def get_aiming_parameters(
    robot_pose: Pose2d,
    field_speeds: ChassisSpeeds,
    real_goal_pose: Pose2d,
    aiming_table: ShooterAimingTable,
    nominal_ball_velocity_mps: float = NOMINAL_BALL_VELOCITY_MPS,
) -> AimingParameters:
    """
    Virtual Goal (SOTM) logic: aim at where the goal will appear based on robot motion and ToF.

    1. Time of flight: dist_to_real_goal / exit_velocity
    2. Virtual goal: G_virtual = G_real - (V_robot * ToF)
    3. Virtual distance and angle from robot to virtual goal
    4. LUT settings for that distance (rpm, hood)
    """
    rx = robot_pose.X()
    ry = robot_pose.Y()
    gx = real_goal_pose.X()
    gy = real_goal_pose.Y()
    dist_to_real = math.hypot(gx - rx, gy - ry)
    if dist_to_real < 1e-6:
        dist_to_real = 1e-6
    tof = dist_to_real / nominal_ball_velocity_mps
    # Virtual goal (look-ahead)
    vgx = gx - (field_speeds.vx * tof)
    vgy = gy - (field_speeds.vy * tof)
    virtual_dist = math.hypot(vgx - rx, vgy - ry)
    virtual_angle = math.atan2(vgy - ry, vgx - rx)
    settings = aiming_table.get_settings(virtual_dist)
    return AimingParameters(
        turret_angle_rad=virtual_angle,
        virtual_dist_m=virtual_dist,
        rps=settings["rpm"],
        hood_rotations=settings["hood"],
    )
