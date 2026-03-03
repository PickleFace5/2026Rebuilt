# Shooting on the Move & Unified Aiming

This document describes the **current** unified kinematic aiming system: it replaces the old zone-based shooter logic with a single **Virtual Goal + LUT** model so the robot can aim and shoot while moving (SOTM) and while stationary.

---

## Current Architecture

### 1. Drivetrain: Field-Relative Velocity

The turret lead for a moving shot uses the robot’s velocity in **field** frame.

- **Location:** `subsystems/swerve/__init__.py`
- **API:** `SwerveSubsystem.get_field_relative_speeds() -> ChassisSpeeds`
- **Logic:** Cached robot-relative chassis speeds are rotated by the current pose rotation via `Translation2d(vx, vy).rotateBy(pose.rotation())`, yielding field-frame vx, vy (omega unchanged).

### 2. Shooter / Hood: Distance LUT (No Zones)

RPM and hood angle come only from a **distance-based lookup table** with linear interpolation between points. There are no zones or polynomial fits.

- **Location:** `subsystems/aiming.py` — `ShooterAimingTable`
- **Storage:** Two internal tables: distance (m) → launcher **RPS** (rotations per second), and distance (m) → hood angle (**rotations**, motor units).
- **API:**
  - `put_rpm(distance_m: float, rps: float)` — add/update one (distance, RPS) sample
  - `put_hood(distance_m: float, hood_rotations: float)` — add/update one (distance, hood rotations) sample
  - `get_settings(distance_m: float) -> {"rpm": rps, "hood": hood_rotations}` — interpolated values for that distance (clamped outside table range)

Out-of-range distances are clamped to the min/max table values. The table is seeded with defaults in `_seed_defaults()` until you replace them with tuned data.

### 3. Virtual Goal Logic (SOTM)

Aiming setpoints are computed from a **virtual goal** so that when the robot is moving, we aim where the goal will be when the ball arrives.

- **Location:** `subsystems/aiming.py` — `get_aiming_parameters(...)`
- **Inputs:** Robot pose (e.g. turret center), field-relative chassis speeds, real goal pose (hub), aiming table, nominal ball velocity (m/s).
- **Steps:**
  1. **Time of flight:** `ToF = distance_to_real_goal / nominal_ball_velocity`
  2. **Virtual goal:** \( G_{virtual} = G_{real} - (\vec{V}_{robot} \times \text{ToF}) \)
  3. **Virtual distance** and **virtual angle** from robot to virtual goal
  4. **LUT lookup:** `aiming_table.get_settings(virtual_dist)` → RPS and hood (rotations)
- **Output:** `AimingParameters(turret_angle_rad, virtual_dist_m, rps, hood_rotations)`

When the robot is stationary, field speed is zero so the virtual goal equals the real goal. Nominal ball velocity is set in `subsystems/aiming.py` as `NOMINAL_BALL_VELOCITY_MPS` (default 12.0 m/s); tune from testing if needed.

### 4. Superstructure Coordination

- **Location:** `subsystems/superstructure.py`
- **When:** Every periodic, when goal is **LAUNCH** or **AIMHUB** and both `aim_pose_supplier` and `aiming_table` are set.
- **Behavior:**
  - Resolves real goal (hub) from alliance (e.g. `Constants.GoalLocations.BLUE_HUB` / `RED_HUB`).
  - Gets field speeds from drivetrain if present; otherwise uses `ChassisSpeeds(0, 0, 0)` (stationary).
  - Calls `get_aiming_parameters(robot_pose, field_speeds, real_goal, aiming_table)`.
  - Pushes setpoints: `turret.set_target_field_angle(...)`, `hood.set_aiming_setpoint(...)`, `launcher.set_aiming_setpoint(...)`.
- **Wiring:** `robot_container.py` builds `aim_pose_supplier` from turret center (via `make_turret_pose_supplier`) and passes `drivetrain`, `aim_pose_supplier`, and a `ShooterAimingTable()` instance into `Superstructure`.

### 5. Subsystems (Turret, Hood, Launcher)

- **Turret:** Uses the field angle from superstructure when set (`set_target_field_angle`); otherwise uses real goal (e.g. for DEPOT/OUTPOST).
- **Hood:** In AIMBOT, uses only the LUT setpoint from superstructure (`set_aiming_setpoint`); fallback if unset is `HoodConstants.STOW`.
- **Launcher:** In SCORE, uses only the LUT RPS from superstructure (`set_aiming_setpoint`); fallback if unset is the base SCORE RPS from state config.

There is **no** remaining zone-based or distance-polynomial logic in hood or launcher; all scoring aim comes from the unified aiming model.

---

## Entering Real Configuration Data From Testing

The aiming system is fully driven by `ShooterAimingTable`. Until you add tuned data, it uses the default seeded table. To use **real** configuration from testing:

### Step 1: Stationary Tuning (Golden Samples)

1. **Setup**
   - Use fresh game pieces and a charged battery (voltage compensation on if you use it).
   - Mark distances from the hub (e.g. 1 m, 2 m, 2.5 m, 3 m, 3.5 m, 4 m). Measure from **turret center** (or the same reference used by `aim_pose_supplier`).

2. **At each distance**
   - Manually adjust **launcher RPS** and **hood angle** until you get 5/5 (or your chosen repeatability) made shots.
   - Record:
     - **Distance** in **meters**.
     - **Launcher RPS** (rotations per second of the flywheel — this is what `LauncherSubsystem` uses, not RPM).
     - **Hood angle** in **rotations** (motor units). If you log in degrees, convert: `hood_rot = (hood_deg / 360)`; if the motor reports rotations, use that directly.

3. **Optional**
   - Estimate **nominal ball exit velocity** (m/s) from distance and time, or from your physics model, and update `NOMINAL_BALL_VELOCITY_MPS` in `subsystems/aiming.py` for better SOTM ToF.

### Step 2: Where to Put the Table Data

The aiming table is created in **`robot_container.py`** when constructing `Superstructure`:

```python
aiming_table=ShooterAimingTable(),
```

To use **your** tuned data:

**Option A — Build a tuned table in `robot_container.py`:**

```python
from subsystems.aiming import ShooterAimingTable

def create_tuned_aiming_table() -> ShooterAimingTable:
    table = ShooterAimingTable()
    # Replace with your (distance_m, RPS, hood_rotations) from testing:
    table.put_rpm(1.0, 28.0)   # 1 m: launcher RPS
    table.put_hood(1.0, 0.002) # 1 m: hood angle (rotations)
    table.put_rpm(2.0, 32.0)
    table.put_hood(2.0, 0.02)
    table.put_rpm(2.55, 32.0)
    table.put_hood(2.55, 0.035)
    table.put_rpm(2.9, 35.0)
    table.put_hood(2.9, 0.036)
    table.put_rpm(3.5, 38.0)
    table.put_hood(3.5, 0.04)
    table.put_rpm(4.0, 40.0)
    table.put_hood(4.0, 0.045)
    return table

# In RobotContainer.__init__, when building Superstructure:
self.superstructure = Superstructure(
    ...
    aiming_table=create_tuned_aiming_table(),
)
```

**Option B — Tuned table in a constants/config module:**

1. Add a module (e.g. `config/aiming_config.py` or in `constants.py`) that defines a function returning a `ShooterAimingTable()` with `put_rpm` / `put_hood` for every (distance_m, RPS, hood_rotations) triplet from your sheet.
2. In `robot_container.py`, import that function and pass `aiming_table=create_tuned_aiming_table()` (or similar) into `Superstructure`.

Use **Option A** for a single robot; use **Option B** if you want to share or switch configs (e.g. by robot or event).

### Step 3: Units Checklist

| Quantity      | Unit    | Notes                                      |
|---------------|---------|--------------------------------------------|
| Distance      | meters  | Same reference as `aim_pose_supplier` (e.g. turret center to hub). |
| Launcher speed| RPS     | Rotations per second (not RPM).            |
| Hood angle    | rotations | Motor units; use same units as hood sensor/encoder. |
| Nominal velocity | m/s  | In `aiming.py`: `NOMINAL_BALL_VELOCITY_MPS`. |

### Step 4: Add More Points

- Add as many (distance, RPS) and (distance, hood) points as you have from testing. The table uses **linear interpolation** between points; more points give smoother behavior.
- You can extend the table beyond your tested range; values outside the min/max distance are clamped to the nearest table value.

---

## Summary

| Component     | Role |
|--------------|------|
| **Drivetrain** | `get_field_relative_speeds()` for SOTM lead. |
| **ShooterAimingTable** | Single source of (distance → RPS, distance → hood rotations); linear interpolation. |
| **get_aiming_parameters()** | Virtual goal + ToF; returns turret angle, virtual dist, RPS, hood rotations. |
| **Superstructure** | Runs aiming when goal is LAUNCH/AIMHUB; pushes setpoints to turret, hood, launcher. |
| **Turret / Hood / Launcher** | Use only the setpoints from superstructure (with minimal fallbacks when unset). |

All aiming for scoring uses this path; there is no legacy zone or distance-polynomial logic left in the codebase.
