"""
Translated from https://github.com/hammerheads5000/FuelSim
v1.0.3
"""
import math
import random
from collections import defaultdict
from dataclasses import dataclass, field
from typing import ClassVar, Optional, Callable

from pykit.logger import Logger
from wpimath.geometry import (Translation3d, Translation2d, Pose2d, Pose3d,
                              Transform3d, Rotation3d, Rotation2d)
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import (seconds, kilograms_per_cubic_meter, meters,
                           kilograms, meters_per_second, radians,
                           meters_per_second_squared)

### Constants
_PERIOD: seconds = 0.02
_GRAVITY: meters_per_second_squared = -9.81
# Room temperature dry air density:
# https://en.wikipedia.org/wiki/Density_of_air#Dry_air
_AIR_DENSITY: kilograms_per_cubic_meter = 1.2041
_FIELD_COR = math.sqrt(22 / 51.5)  # Coefficient of restitution with the field
_FUEL_COR = 0.5  # Coefficient of restitution with another fuel
_NET_COR = 0.2  # Coefficient of restitution with the net
_ROBOT_COR = 0.1  # Coefficient of restitution with a robot
_FUEL_RADIUS: meters = 0.075
_FIELD_LENGTH: meters = 16.51
_FIELD_WIDTH: meters = 8.04
_TRENCH_WIDTH: meters = 1.265
_TRENCH_BLOCK_WIDTH: meters = 0.305
_TRENCH_HEIGHT: meters = 0.565
_TRENCH_BAR_HEIGHT: meters = 0.102
_TRENCH_BAR_WIDTH: meters = 0.152
_FRICTION = 0.1  # Proportion of horizontal vel to lose per sec while on ground
_FUEL_MASS: kilograms = 0.448 * 0.45392
_FUEL_CROSS_AREA = math.pi * _FUEL_RADIUS ** 2
# Drag coefficient of smooth sphere
# https://en.wikipedia.org/wiki/Drag_coefficient#/media/File:14ilf1l.svg
_DRAG_COF = 0.47  # dimensionless
_DRAG_FORCE_FACTOR = 0.5 * _AIR_DENSITY * _DRAG_COF * _FUEL_CROSS_AREA

_FIELD_XZ_LINES: tuple[tuple[Translation3d, Translation3d], ...] = (
    (Translation3d(0, 0, 0), Translation3d(_FIELD_LENGTH, _FIELD_WIDTH, 0)),

    (Translation3d(3.96, 1.57, 0),
     Translation3d(4.61, _FIELD_WIDTH / 2 - 0.60, 0.165)),

    (Translation3d(3.96, _FIELD_WIDTH / 2 + 0.60, 0),
     Translation3d(4.61, _FIELD_WIDTH - 1.57, 0.165)),

    (Translation3d(4.61, 1.57, 0.165),
     Translation3d(5.18, _FIELD_WIDTH / 2 - 0.60, 0)),

    (Translation3d(4.61, _FIELD_WIDTH / 2 + 0.60, 0.165),
     Translation3d(5.18, _FIELD_WIDTH - 1.57, 0)),

    (Translation3d(_FIELD_LENGTH - 5.18, 1.57, 0),
     Translation3d(_FIELD_LENGTH - 4.61, _FIELD_WIDTH / 2 - 0.60, 0.165)),

    (Translation3d(_FIELD_LENGTH - 5.18, _FIELD_WIDTH / 2 + 0.60, 0),
     Translation3d(_FIELD_LENGTH - 4.61, _FIELD_WIDTH - 1.57, 0.165)),

    (Translation3d(_FIELD_LENGTH - 4.61, 1.57, 0.165),
     Translation3d(_FIELD_LENGTH - 3.96, _FIELD_WIDTH / 2 - 0.60, 0)),

    (Translation3d(_FIELD_LENGTH - 4.61, _FIELD_WIDTH / 2 + 0.60, 0.165),
     Translation3d(_FIELD_LENGTH - 3.96, _FIELD_WIDTH - 1.57, 0)),

    (Translation3d(3.96, _TRENCH_WIDTH, _TRENCH_HEIGHT), Translation3d(
        5.18, _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH, _TRENCH_HEIGHT
    )),

    (Translation3d(3.96, _FIELD_WIDTH - 1.57, _TRENCH_HEIGHT), Translation3d(
        5.18, _FIELD_WIDTH - 1.57 + _TRENCH_BLOCK_WIDTH, _TRENCH_HEIGHT
    )),

    (Translation3d(_FIELD_LENGTH - 5.18, _TRENCH_WIDTH, _TRENCH_HEIGHT),
     Translation3d(
         _FIELD_LENGTH - 3.96,
         _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH,
         _TRENCH_HEIGHT
     )),

    (Translation3d(_FIELD_LENGTH - 5.18, _FIELD_WIDTH - 1.57, _TRENCH_HEIGHT),
     Translation3d(
         _FIELD_LENGTH - 3.96,
         _FIELD_WIDTH - 1.57 + _TRENCH_BLOCK_WIDTH,
         _TRENCH_HEIGHT
     )),

    (Translation3d(
        4.61 - _TRENCH_BAR_WIDTH / 2, 0, _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
    ), Translation3d(
        4.61 + _TRENCH_BAR_WIDTH / 2,
        _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH,
        _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
    )),

    (Translation3d(
        4.61 - _TRENCH_BAR_WIDTH / 2,
        _FIELD_WIDTH - 1.57,
        _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
    ), Translation3d(
        4.61 + _TRENCH_BAR_WIDTH / 2,
        _FIELD_WIDTH,
        _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
    )),

    (Translation3d(
        _FIELD_LENGTH - 4.61 - _TRENCH_BAR_WIDTH / 2,
        0,
        _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
    ), Translation3d(
        _FIELD_LENGTH - 4.61 + _TRENCH_BAR_WIDTH / 2,
        _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH,
        _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
    )),

    (Translation3d(
        _FIELD_LENGTH - 4.61 - _TRENCH_BAR_WIDTH / 2,
        _FIELD_WIDTH - 1.57,
        _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
    ), Translation3d(
        _FIELD_LENGTH - 4.61 + _TRENCH_BAR_WIDTH / 2,
        _FIELD_WIDTH,
        _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
    )))


@dataclass
class Hub:
    """Handles hub interactions with fuel and collisions."""
    center: Translation2d
    exit: Translation3d
    exit_vel_x_mult: int

    _score: int = field(default=0, init=False, repr=False)

    _ENTRY_HEIGHT: ClassVar[float] = 1.83
    _ENTRY_RADIUS: ClassVar[float] = 0.56
    _SIDE: ClassVar[float] = 1.2
    _NET_HEIGHT_MAX: ClassVar[float] = 3.057
    _NET_HEIGHT_MIN: ClassVar[float] = 1.5
    _NET_OFFSET: ClassVar[float] = _SIDE / 2 + 0.261
    _NET_WIDTH: ClassVar[float] = 1.484

    def handle_hub_interaction(self, fuel: "Fuel", subticks: int) -> None:
        """Score and release fuel if needed."""
        if self._did_fuel_score(fuel, subticks):
            fuel.pos = self.exit
            fuel.vel = self._get_dispersal_velocity()
            self._score += 1

    def _did_fuel_score(self, fuel: "Fuel", subticks: int) -> bool:
        """Check if fuel is withing entry bounds."""
        return fuel.pos.toTranslation2d().distance(
            self.center
        ) <= self._ENTRY_RADIUS and fuel.pos.z <= self._ENTRY_HEIGHT < (
                fuel.pos - (fuel.vel * (_PERIOD / subticks))).z

    def _get_dispersal_velocity(self) -> Translation3d:
        """Calculate random release velocity."""
        return Translation3d(
            self.exit_vel_x_mult * (random.random() + 0.1) * 1.5,
            random.uniform(-1, 1),
            0
        )

    @property
    def score(self) -> int:
        """Current count of fuel scored in this hub"""
        return self._score

    def reset_score(self) -> None:
        """Reset this hub's score to 0"""
        self._score = 0

    def fuel_collide_side(self, fuel: "Fuel") -> None:
        """Side rectangles for collision checks."""
        _fuel_collide_rectangle(
            fuel, Translation3d(
                self.center.x - self._SIDE / 2,
                self.center.y - self._SIDE / 2,
                0
            ), Translation3d(
                self.center.x + self._SIDE / 2,
                self.center.y + self._SIDE / 2,
                self._ENTRY_HEIGHT - 0.1
            )
        )

    def fuel_hit_net(self, fuel: "Fuel") -> float:
        """Checks if the fuel hits the net."""
        if (
                fuel.pos.z > self._NET_HEIGHT_MAX or fuel.pos.z <
                self._NET_HEIGHT_MIN):
            return 0
        if (
                fuel.pos.y > self.center.y + self._NET_WIDTH / 2 or
                fuel.pos.y < self.center.y - self._NET_WIDTH / 2):
            return 0
        if (
                fuel.pos.x > self.center.x + self._NET_OFFSET *
                self.exit_vel_x_mult):
            return max(
                0.0,
                self.center.x + self._NET_OFFSET * self.exit_vel_x_mult - (
                        fuel.pos.x - _FUEL_RADIUS)
            )
        return min(
            0.0,
            self.center.x + self._NET_OFFSET * self.exit_vel_x_mult - (
                    fuel.pos.x + _FUEL_RADIUS)
        )


BLUE_HUB = Hub(
    Translation2d(4.61, _FIELD_WIDTH / 2),
    Translation3d(5.3, _FIELD_WIDTH / 2, 0.89),
    1
)
RED_HUB = Hub(
    Translation2d(_FIELD_LENGTH - 4.61, _FIELD_WIDTH / 2),
    Translation3d(_FIELD_LENGTH - 5.3, _FIELD_WIDTH / 2, 0.89),
    -1
)


@dataclass(slots=True)
class Fuel:
    """Fuel dataclass"""
    pos: Translation3d
    vel: Translation3d = field(default_factory=Translation3d)

    # pylint: disable=too-many-locals
    def update(self, simulate_air_resistance: bool, subticks: int) -> None:
        """Update position, air resistance, and collisions."""
        dt = _PERIOD / subticks

        vx, vy, vz = self.vel.x, self.vel.y, self.vel.z
        px, py, pz = self.pos.x, self.pos.y, self.pos.z

        px += vx * dt
        py += vy * dt
        pz += vz * dt
        if pz > _FUEL_RADIUS:
            fg_z = _GRAVITY * _FUEL_MASS
            drag_z = 0.0

            if simulate_air_resistance:
                speed2 = vx * vx + vy * vy + vz * vz
                if speed2 > 1e-12:
                    speed = speed2 ** 0.5
                    drag_x = -_DRAG_FORCE_FACTOR * speed * vx
                    drag_y = -_DRAG_FORCE_FACTOR * speed * vy
                    drag_z = -_DRAG_FORCE_FACTOR * speed * vz

                    ax = drag_x / _FUEL_MASS
                    ay = drag_y / _FUEL_MASS
                    vx += ax * dt
                    vy += ay * dt

            az = (fg_z + drag_z) / _FUEL_MASS
            vz += az * dt

        # Ground contact + friction
        if abs(vz) < 0.05 and pz <= _FUEL_RADIUS + 0.03:
            vz = 0.0
            friction_factor = 1 - _FRICTION * dt
            vx *= friction_factor
            vy *= friction_factor

        self.pos = Translation3d(px, py, pz)
        self.vel = Translation3d(vx, vy, vz)

        # Collision handling
        if vx * vx + vy * vy > 1e-12:
            self._handle_field_collisions(subticks)

    def _handle_xz_line_collision(self,
                                  line_start: Translation3d,
                                  line_end: Translation3d
                                  ) -> None:
        """Handle a lotta collisions."""
        if self.pos.y < line_start.y or self.pos.y > line_end.y:
            return
        # Convert into 2D
        start2d = Translation2d(line_start.x, line_start.z)
        end2d = Translation2d(line_end.x, line_end.z)
        pos2d = Translation2d(self.pos.x, self.pos.z)
        line_vec = end2d - start2d

        # Get the closest point on the line
        projected = start2d + (line_vec * (pos2d - start2d).dot(
            line_vec
        ) / line_vec.squaredNorm())

        if projected.distance(start2d) + projected.distance(
                end2d
        ) > line_vec.norm():
            return  # projected point not on the line
        dist = pos2d.distance(projected)
        if dist > _FUEL_RADIUS:
            return  # not intersecting line
        # Back into 3D
        normal = Translation3d(
            -line_vec.y,
            0,
            line_vec.x
        ) / line_vec.norm()

        # Apply collision response
        self.pos += normal * (_FUEL_RADIUS - dist)
        if self.vel.dot(normal) > 0:
            return  # already moving away from line
        self.vel -= normal * (1 + _FIELD_COR) * self.vel.dot(normal)

    def _handle_field_collisions(self, subticks: int) -> None:
        """Self-explanatory."""
        # floor and bumps
        if self.vel.norm() < 1e-6:
            return  # No checks if we aren't moving
        for _, line in enumerate(_FIELD_XZ_LINES):
            self._handle_xz_line_collision(line[0], line[1])

        # edges
        if self.pos.x < _FUEL_RADIUS and self.vel.x < 0:
            self.pos += Translation3d(_FUEL_RADIUS - self.pos.x, 0, 0)
            self.vel += Translation3d(-(1 + _FIELD_COR) * self.vel.x, 0, 0)
        elif self.pos.x > _FIELD_LENGTH - _FUEL_RADIUS and self.vel.x > 0:
            self.pos += Translation3d(
                _FIELD_LENGTH - _FUEL_RADIUS - self.pos.x, 0, 0
            )
            self.vel += Translation3d(-(1 + _FIELD_COR) * self.vel.x, 0, 0)

        if self.pos.y < _FUEL_RADIUS and self.vel.y < 0:
            self.pos += Translation3d(0, _FUEL_RADIUS - self.pos.y, 0)
            self.vel += Translation3d(0, -(1 + _FIELD_COR) * self.vel.y, 0)
        elif self.pos.y > _FIELD_WIDTH - _FUEL_RADIUS and self.vel.y > 0:
            self.pos += Translation3d(
                0, _FIELD_WIDTH - _FUEL_RADIUS - self.pos.y, 0
            )
            self.vel += Translation3d(0, -(1 + _FIELD_COR) * self.vel.y, 0)

        # Hubs
        if self.pos.x < _FIELD_LENGTH / 2:
            self._handle_hub_collisions(BLUE_HUB, subticks)
        else:
            self._handle_hub_collisions(RED_HUB, subticks)

        self._handle_trench_collisions()

    def _handle_hub_collisions(self, hub: "Hub", subticks: int) -> None:
        """Lots of collision checks..."""
        hub.handle_hub_interaction(self, subticks)
        hub.fuel_collide_side(self)

        net_collision = hub.fuel_hit_net(self)
        if net_collision != 0:
            self.pos += Translation3d(net_collision, 0, 0)
            self.vel = Translation3d(
                -self.vel.x * _NET_COR, self.vel.y * _NET_COR, self.vel.z
            )

    def _handle_trench_collisions(self) -> None:
        """Baseball, huh?"""
        _fuel_collide_rectangle(
            self, Translation3d(3.96, _TRENCH_WIDTH, 0), Translation3d(
                5.18, _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH, _TRENCH_HEIGHT
            )
        )
        _fuel_collide_rectangle(
            self, Translation3d(3.96, _FIELD_WIDTH - 1.57, 0), Translation3d(
                5.18, _FIELD_WIDTH - 1.57 + _TRENCH_BLOCK_WIDTH, _TRENCH_HEIGHT
            )
        )
        _fuel_collide_rectangle(
            self,
            Translation3d(_FIELD_LENGTH - 5.18, _TRENCH_WIDTH, 0),
            Translation3d(
                _FIELD_LENGTH - 3.96,
                _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH,
                _TRENCH_HEIGHT
            )
        )
        _fuel_collide_rectangle(
            self,
            Translation3d(_FIELD_LENGTH - 5.18, _FIELD_WIDTH - 1.57, 0),
            Translation3d(
                _FIELD_LENGTH - 3.96,
                _FIELD_WIDTH - 1.57 + _TRENCH_BLOCK_WIDTH,
                _TRENCH_HEIGHT
            )
        )
        _fuel_collide_rectangle(
            self,
            Translation3d(4.61 - _TRENCH_BAR_WIDTH / 2, 0, _TRENCH_HEIGHT),
            Translation3d(
                4.61 + _TRENCH_BAR_WIDTH / 2,
                _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH,
                _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
            )
        )
        _fuel_collide_rectangle(
            self, Translation3d(
                4.61 - _TRENCH_BAR_WIDTH / 2,
                _FIELD_WIDTH - 1.57,
                _TRENCH_HEIGHT
            ), Translation3d(
                4.61 + _TRENCH_BAR_WIDTH / 2,
                _FIELD_WIDTH,
                _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
            )
        )
        _fuel_collide_rectangle(
            self, Translation3d(
                _FIELD_LENGTH - 4.61 - _TRENCH_BAR_WIDTH / 2, 0, _TRENCH_HEIGHT
            ), Translation3d(
                _FIELD_LENGTH - 4.61 + _TRENCH_BAR_WIDTH / 2,
                _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH,
                _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
            )
        )
        _fuel_collide_rectangle(
            self, Translation3d(
                _FIELD_LENGTH - 4.61 - _TRENCH_BAR_WIDTH / 2,
                _FIELD_WIDTH - 1.57,
                _TRENCH_HEIGHT
            ), Translation3d(
                _FIELD_LENGTH - 4.61 + _TRENCH_BAR_WIDTH / 2,
                _FIELD_WIDTH,
                _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
            )
        )

    def add_impulse(self, impulse: Translation3d) -> None:
        """Update impulse for fuel velocity."""
        self.vel += impulse


def _fuel_collide_rectangle(fuel: Fuel,
                            start: Translation3d,
                            end: Translation3d
                            ) -> None:
    """Simple rectangle collision check."""
    if (fuel.vel.norm() < 1e-6 or
            fuel.pos.z > end.z + _FUEL_RADIUS or
            fuel.pos.z < start.z - _FUEL_RADIUS):
        return  # above rectangle or not moving
    distance_to_left = start.x - _FUEL_RADIUS - fuel.pos.x
    distance_to_right = fuel.pos.x - end.x - _FUEL_RADIUS
    distance_to_top = fuel.pos.y - end.y - _FUEL_RADIUS
    distance_to_bottom = start.y - _FUEL_RADIUS - fuel.pos.y

    # not inside hub
    if (
            distance_to_left > 0 or distance_to_right > 0 or distance_to_top
            > 0
            or distance_to_bottom > 0):
        return

    # Find minimum distance to side and send corresponding collision response
    if fuel.pos.x < start.x or (
            distance_to_left >= distance_to_right and distance_to_left >=
            distance_to_top and distance_to_left >= distance_to_bottom):
        collision = Translation2d(distance_to_left, 0)
    elif fuel.pos.x >= end.x or (
            distance_to_right >= distance_to_left and distance_to_right >=
            distance_to_top and distance_to_right >= distance_to_bottom):
        collision = Translation2d(-distance_to_right, 0)
    elif fuel.pos.y > end.y or (
            distance_to_top >= distance_to_left and distance_to_top >=
            distance_to_right and distance_to_top >= distance_to_bottom):
        collision = Translation2d(0, -distance_to_top)
    else:
        collision = Translation2d(0, distance_to_bottom)

    if collision.x != 0:
        fuel.pos += Translation3d(collision)
        fuel.vel += Translation3d(-(1 + _FIELD_COR) * fuel.vel.x, 0, 0)
    elif collision.y != 0:
        fuel.pos += Translation3d(collision)
        fuel.vel += Translation3d(0, -(1 + _FIELD_COR) * fuel.vel.y, 0)


@dataclass
class SimIntake:
    """Simulated intake."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    able_to_intake: Callable[[], bool] = field(default=lambda: True)
    callback: Callable[[], None] = field(default=lambda: None)

    def should_intake(self,
                      fuel: Fuel,
                      robot_pose: Pose2d,
                      bumper_height: meters
                      ) -> bool:
        """Check if we're able to intake the fuel."""
        if not self.able_to_intake() or fuel.pos.z > bumper_height:
            return False

        fuel_relative_pos = Pose2d(
            fuel.pos.toTranslation2d(), Rotation2d()
        ).relativeTo(robot_pose).translation()

        result = (
                self.x_min <= fuel_relative_pos.x <= self.x_max and
                self.y_min <= fuel_relative_pos.y <= self.y_max)
        if result:
            self.callback()
        return result


def _handle_fuel_collision(a: Fuel, b: Fuel) -> None:
    """Collision."""
    normal = a.pos - b.pos
    distance = normal.norm()
    if distance == 0:
        normal = Translation3d(1, 0, 0)
        distance = 1
    normal = normal / distance
    impulse = 0.5 * (1 + _FUEL_COR) * (b.vel - a.vel).dot(normal)
    intersection = _FUEL_RADIUS * 2 - distance
    a.pos = a.pos + normal * (intersection / 2)
    b.pos = b.pos - normal * (intersection / 2)
    a.add_impulse(normal * impulse)
    b.add_impulse(normal * -impulse)


_CELL_SIZE = 0.25
_GRID_COLS = math.ceil(_FIELD_LENGTH / _CELL_SIZE)
_GRID_ROWS = math.ceil(_FIELD_WIDTH / _CELL_SIZE)


# pylint: disable=too-many-instance-attributes
class FuelSim:
    """Handles all fuel."""

    def __init__(self, table_key: str = "Fuel Simulation/") -> None:
        self._grid: dict[tuple[int, int], list[Fuel]] = defaultdict(list)
        self.fuels: list[Fuel] = []
        self.running: bool = False
        self.simulate_air_resistance: bool = False
        self.subticks: int = 5
        self.intakes: list[SimIntake] = []
        self._table_key = table_key

        self.robot_pose_supplier: Optional[Callable[[], Pose2d]] = None
        self.robot_speeds_supplier: Optional[
            Callable[[], ChassisSpeeds]] = None
        self.robot_width: float = 0
        self.robot_length: float = 0
        self.bumper_height: float = 0

    def clear_fuel(self) -> None:
        """Clears the field of fuel"""
        self.fuels.clear()

    def spawn_starting_fuel(self) -> None:
        """Spawns fuel in neutral zone and depots."""
        # Center fuel
        center = Translation3d(
            _FIELD_LENGTH / 2,
            _FIELD_WIDTH / 2,
            _FUEL_RADIUS
            )
        self.fuels += [
            Fuel(
                center + Translation3d(
                    x * (0.076 + 0.152 * j),
                    y * (0.0254 + 0.076 + 0.152 * i),
                    0
                )
            )
            for i in range(15)
            for j in range(6)
            for x, y in [(1, 1), (-1, 1), (1, -1), (-1, -1)]
        ]

    def spawn_depot_fuel(self) -> None:
        """Adds depot fuel."""
        for i in range(3):
            for j in range(4):
                self.fuels.append(
                    Fuel(
                        Translation3d(
                            0.076 + 0.152 * j,
                            5.95 + 0.076 + 0.152 * i,
                            _FUEL_RADIUS
                        )
                    )
                )
                self.fuels.append(
                    Fuel(
                        Translation3d(
                            0.076 + 0.152 * j,
                            5.95 - 0.076 - 0.152 * i,
                            _FUEL_RADIUS
                        )
                    )
                )
                self.fuels.append(
                    Fuel(
                        Translation3d(
                            _FIELD_LENGTH - 0.076 - 0.152 * j,
                            2.09 + 0.076 + 0.152 * i,
                            _FUEL_RADIUS
                        )
                    )
                )
                self.fuels.append(
                    Fuel(
                        Translation3d(
                            _FIELD_LENGTH - 0.076 - 0.152 * j,
                            2.09 - 0.076 - 0.152 * i,
                            _FUEL_RADIUS
                        )
                    )
                )

    def spawn_less_starting_fuel(self) -> None:
        """Spawns less fuel in the neutral zone for performance’s sake."""
        # Center fuel
        center = Translation3d(
            _FIELD_LENGTH / 2,
            _FIELD_WIDTH / 2,
            _FUEL_RADIUS
            )
        self.fuels += [
            Fuel(
                center + Translation3d(
                    x * (0.076 + 0.152 * j),
                    y * (0.0254 + 0.076 + 0.152 * i),
                    0
                )
            )
            for i in range(3)
            for j in range(2)
            for x, y in [(1, 1), (-1, 1), (1, -1), (-1, -1)]
        ]

    def _log_fuels(self) -> None:
        """Adds array of `Translation3d`'s to NetworkTables at tableKey +
        "/Fuels"""
        Logger.recordOutput(
            f"{self._table_key}/Fuel", [fuel.pos for fuel in self.fuels]
        )

    def start(self) -> None:
        """Start the simulation. `updateSim` must still be called every loop"""
        self.running = True

    def stop(self) -> None:
        """Pause the simulation."""
        self.running = False

    def enable_air_resistance(self) -> None:
        """Enables accounting for drag force in physics step"""
        self.simulate_air_resistance = True

    def set_subticks(self, subticks: int) -> None:
        """Sets the number of physics iterations per loop (0.02s)"""
        self.subticks = subticks

    # pylint: disable=too-many-arguments, too-many-positional-arguments
    def register_robot(self,
                       width: meters,
                       length: meters,
                       bumper_height: meters,
                       pose_supplier: Callable[[], Pose2d],
                       field_speeds_supplier: Callable[[], ChassisSpeeds]
                       ) -> None:
        """Registers a robot with the fuel simulator"""
        self.robot_pose_supplier = pose_supplier
        self.robot_speeds_supplier = field_speeds_supplier
        self.robot_width = width
        self.robot_length = length
        self.bumper_height = bumper_height

    def update_sim(self) -> None:
        """
        To be called periodically
        Will do nothing if sim is not running
        """
        if self.running:
            self.step_sim()

    def step_sim(self) -> None:
        """Run the simulation forward 1 time step (0.02s)"""
        for _ in range(self.subticks):
            for fuel in self.fuels:
                fuel.update(self.simulate_air_resistance, self.subticks)

            self._handle_fuel_collisions(self.fuels)

            if self.robot_pose_supplier is not None:
                self._handle_robot_collisions(self.fuels)
                self._handle_intakes(self.fuels)

        self._log_fuels()

    def spawn_fuel(self, pos: Translation3d, vel: Translation3d) -> None:
        """Adds a fuel onto the field"""
        self.fuels.append(Fuel(pos, vel))

    def launch_fuel(self,
                    launch_velocity: meters_per_second,
                    hood_angle: radians,
                    turret_yaw: radians,
                    launch_height: meters
                    ) -> None:
        """Shoots fuel at the desired angles and velocity."""
        if (
                self.robot_pose_supplier is None or
                self.robot_speeds_supplier is None):
            raise RuntimeError(
                "Robot must be registered before launching fuel."
            )

        launch_pose = Pose3d(self.robot_pose_supplier()) + Transform3d(
            Translation3d(0, 0, launch_height), Rotation3d()
        )
        field_speeds = self.robot_speeds_supplier()

        horizontal_vel = math.cos(hood_angle) * launch_velocity
        vertical_vel = math.sin(hood_angle) * launch_velocity

        yaw = turret_yaw + launch_pose.rotation().z
        x_vel = horizontal_vel * math.cos(yaw)
        y_vel = horizontal_vel * math.sin(yaw)
        x_vel += field_speeds.vx
        y_vel += field_speeds.vy

        self.spawn_fuel(
            launch_pose.translation(),
            Translation3d(x_vel, y_vel, vertical_vel)
        )

    def _handle_robot_collision(self,
                                fuel: Fuel,
                                robot: Pose2d,
                                robot_vel: Translation2d
                                ) -> None:
        """Handle a single robot to fuel collision."""
        if fuel.pos.toTranslation2d().distance(
                robot.translation()
        ) > self.robot_length:
            return
        relative_pos = Pose2d(
            fuel.pos.toTranslation2d(), Rotation2d()
        ).relativeTo(robot).translation()

        if fuel.pos.z > self.bumper_height:
            return  # above bumpers
        distance_to_bottom = (
                -_FUEL_RADIUS - self.robot_length / 2 - relative_pos.x)
        distance_to_top = (-_FUEL_RADIUS - self.robot_length / 2 +
                           relative_pos.x)
        distance_to_right = (-_FUEL_RADIUS - self.robot_width / 2 -
                             relative_pos.y)
        distance_to_left = (-_FUEL_RADIUS - self.robot_width / 2 +
                            relative_pos.y)

        # not inside robot
        if (
                distance_to_bottom > 0 or distance_to_top > 0 or
                distance_to_right
                > 0 or distance_to_left > 0):
            return

        # Find minimum distance to side and send corresponding collision
        # response
        if (
                distance_to_bottom >= distance_to_top and distance_to_bottom >=
                distance_to_right and distance_to_bottom >= distance_to_left):
            pos_offset = Translation2d(distance_to_bottom, 0)
        elif (
                distance_to_top >= distance_to_bottom and distance_to_top >=
                distance_to_right and distance_to_top >= distance_to_left):
            pos_offset = Translation2d(-distance_to_top, 0)
        elif (
                distance_to_right >= distance_to_bottom and
                distance_to_right >=
                distance_to_top and distance_to_right >= distance_to_left):
            pos_offset = Translation2d(0, distance_to_right)
        else:
            pos_offset = Translation2d(0, -distance_to_left)

        pos_offset = pos_offset.rotateBy(robot.rotation())
        fuel.pos += Translation3d(pos_offset)
        normal = pos_offset / pos_offset.norm()
        if fuel.vel.toTranslation2d().dot(normal) < 0:
            fuel.add_impulse(
                Translation3d(
                    normal * (-fuel.vel.toTranslation2d().dot(normal) * (
                            1 + _ROBOT_COR))
                )
            )
        if robot_vel.dot(normal) > 0:
            fuel.add_impulse(Translation3d(normal * robot_vel.dot(normal)))

    def _handle_robot_collisions(self, fuels: list[Fuel]) -> None:
        """Plural."""
        if (self.robot_pose_supplier is None or self.robot_speeds_supplier is
                None):
            return
        robot = self.robot_pose_supplier()
        speeds = self.robot_speeds_supplier()
        robot_vel = Translation2d(speeds.vx, speeds.vy)

        for fuel in fuels:
            self._handle_robot_collision(fuel, robot, robot_vel)

    def _handle_intakes(self, fuels: list[Fuel]) -> None:
        """Update intakes."""
        if not self.robot_pose_supplier:
            return
        robot = self.robot_pose_supplier()
        for intake in self.intakes:
            for i in reversed(range(len(fuels))):
                if intake.should_intake(fuels[i], robot, self.bumper_height):
                    fuels.pop(i)

    def _handle_fuel_collisions(self, fuels: list[Fuel]) -> None:
        """There is so many freaking collision calls."""
        # Clear grid
        self._grid.clear()

        # Populate grid
        for fuel in fuels:
            col = int(fuel.pos.x / _CELL_SIZE)
            row = int(fuel.pos.y / _CELL_SIZE)

            if 0 <= col < _GRID_COLS and 0 <= row < _GRID_ROWS:
                self._grid[col, row].append(fuel)

        # Check collisions
        # pylint: disable=too-many-nested-blocks
        for fuel in fuels:
            col = int(fuel.pos.x / _CELL_SIZE)
            row = int(fuel.pos.y / _CELL_SIZE)

            for i in range(col - 1, col + 2):
                for j in range(row - 1, row + 2):
                    if 0 <= i < _GRID_COLS and 0 <= j < _GRID_ROWS:
                        for other in self._grid.get((i, j), []):
                            if (fuel is not other and fuel.pos.distance(
                                    other.pos
                            ) < _FUEL_RADIUS * 2 and id(fuel) < id(other)):
                                _handle_fuel_collision(fuel, other)

    def register_intake(self,
                        x_min: float,
                        x_max: float,
                        y_min: float,
                        y_max: float,
                        able_to_intake: Callable[[], bool] = lambda: True,
                        callback: Callable[[], None] = lambda: None
                        ) -> None:
        """Register an intake."""
        self.intakes.append(
            SimIntake(x_min, x_max, y_min, y_max, able_to_intake, callback)
        )
