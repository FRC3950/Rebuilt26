# Code Architecture Reference

Use this to map robot behavior to code paths before editing.

## Project Shape

- Java WPILib command-based robot.
- GradleRIO 2026 project with AdvantageKit logging.
- CTRE Phoenix 6, REVLib, PathPlannerLib, AdvantageKit, and PhotonVision vendordeps are present.
- Toolchain snapshot from this checkout: GradleRIO `2026.2.1`, Phoenix 6 `26.1.3`, REVLib `2026.0.5`, PathPlannerLib `2026.1.2`, AdvantageKit `26.0.2`.
- `Robot.java` stays thin; `RobotContainer` constructs subsystems, controls, autos, code-mode selection, and named commands.
- Hardware code follows an IO pattern:
  - subsystem owns behavior and logging
  - `*IO` defines hardware interface
  - `*IOTalonFX`, `*IOServoHub`, or `*IOLimelight` talks to real hardware
  - no-op IO supports replay/unavailable hardware paths

## Where To Search First

| Physical area | Code to inspect |
|---|---|
| Robot composition, autos, code mode | `RobotContainer.java` |
| Drivebase, PathPlanner, odometry | `subsystems/drive/Drive.java`, `Module*`, `Gyro*`, `generated/TunerConstants.java` |
| Driver control and bindings | `commands/DriveCommands.java`, `controls/CompBindings.java`, `controls/CrazyBindings.java`, `controls/DemoContainer.java` |
| Intake deploy/roller | `commands/IntakeCommand.java`, `subsystems/intake/*`, `Constants.SubsystemConstants.Intake`, binding files |
| Hotdogs/indexer/feed | `subsystems/indexer/*`, `RobotContainer` named commands, binding files |
| Turret command surface | `subsystems/turret/Turret.java`, `TurretTargeting.java`, `GetAdjustedShot.java` |
| Turret hardware | `turret_base/azimuth/*`, `turret_base/flywheels/*`, `turret_base/hood/*` |
| Shot tuning | `src/main/deploy/shot_table.json`, `util/Distancer.java`, `GetAdjustedShot.java` |
| Vision pose | `subsystems/vision/*`, `VisionConstants.java`, `Drive.addVisionMeasurement()` |
| Field transforms | `Constants.FieldConstants`, `util/AllianceFlipUtil.java`, `util/Zones.java` |
| Autos and event markers | `src/main/deploy/pathplanner/autos`, `src/main/deploy/pathplanner/paths`, `RobotContainer` `NamedCommands` |

## Subsystems

- `Drive`: four-module swerve, pose estimator, PathPlanner `AutoBuilder`, PathPlanner logging, SysId hooks, and vision measurement fusion.
- `Intake`: pivot setpoint, roller velocity command, intake state logging, and simple extend/retract/on/off commands.
- `Indexer`: two controlled outputs: indexer motor and hotdog motor. Forward feed can be requested while applied output is gated by turret flywheel readiness. Any forward hotdog command can auto-unjam on hotdog stator current by stopping the indexer and reversing the hotdog for `0.15 s`.
- `Turret`: combines azimuth, hood, and flywheels. It clamps hood angle, applies per-turret flywheel/TOF/turn fudge factors, selects a safe azimuth wrap within limits, commands azimuth position plus velocity, and zeroes azimuth while disabled.
- `Vision`: accepts pose observations from both cameras, rejects some impossible observations, and forwards accepted poses into drive odometry.

## Commands And Controls

- `IntakeCommand` extends the intake and starts the roller. It does not run the indexer or hotdogs.
- `RobotContainer.CodeMode` is a dashboard-selected top-level control mode: `COMPETITION` or `DEMO`.
- Code mode can only be applied while disabled. If the chooser changes while enabled, selected telemetry can change but applied controls do not.
- `CompBindings` owns the normal driver/operator split.
- `CrazyBindings` ports major actions to one driver controller.
- `DemoContainer` is created lazily only when `Code Mode = Demo` is applied. It owns demo-only dashboard controls and its own binding chooser.
- Demo binding mode defaults to crazy bindings and can switch back to competition bindings while disabled.
- Turrets have default `TurretTargeting` commands in competition defaults.
- `Turret.toggleTurretMode()` changes a static targeting mode used by both turrets.
- Competition driver controller is port `0`; operator controller is port `1`.
- Driver controls swerve, heading reset, and hub-facing drive lock.
- Operator controls intake, retract, feed/shoot, unjam/reverse, and turret mode toggle.
- Crazy bindings move major actions to the driver controller only.
- Demo mode does not create new mechanism behavior yet; it chooses between `CrazyBindings` and `CompBindings` and applies demo drive-speed limits.

## Code Mode And Demo Dashboard

| Dashboard key | Owner | Meaning |
|---|---|---|
| `Code Mode` | `RobotContainer` | Top-level chooser: `Competition` default, `Demo` option |
| `Code Mode/Selected` | `RobotContainer` | Last selected top-level code mode |
| `Code Mode/Applied` | `RobotContainer` | Active top-level code mode after disabled-only gate |
| `Demo Mode/Bindings` | `DemoContainer` | Demo-only chooser: `Crazy` default, `Competition` option |
| `Demo Mode/Bindings Selected` | `DemoContainer` | Last selected demo binding mode |
| `Demo Mode/Bindings Applied` | `DemoContainer` | Active demo binding mode after disabled-only gate |
| `Demo Mode/Max Speed MPS` | `DemoContainer` | Demo-only overall drive speed cap, default `2.0 m/s` |
| `Demo Mode/Reduced Speed While Shooting MPS` | `DemoContainer` | Demo-only intake/feed drive cap, defaulting to the competition reduced speed |

Demo dashboard entries should not appear during normal competition startup. They are initialized by `DemoContainer`, so they are published only after Demo mode is applied once. Once published, NetworkTables/SmartDashboard may keep showing them after switching back to Competition.

Competition mode resets `Drive` to the physical max speed supplier and the fixed `Constants.SubsystemConstants.Drive.reducedSpeed`. Demo mode swaps in dashboard-backed suppliers. Demo tuning must not change competition behavior.

## Important Trigger Bindings

| Mode/input | Behavior |
|---|---|
| Driver left stick + right X | Default swerve drive |
| Driver `Y` | Reset current pose rotation to zero while preserving translation |
| Driver `A` held | Drive while facing the hub target |
| Operator left trigger held | `IntakeCommand`: extend intake and run roller |
| Operator right bumper | Retract intake |
| Operator right trigger held | Request forward feed; indexer and hotdogs apply only when both turret flywheels are ready |
| Operator `B` held | Reverse intake and hotdog for unjam |
| Operator `A` | Toggle shared turret targeting lock mode |
| Robot in neutral zone | Both turrets target closer ferry target instead of hub |
| Demo `Code Mode = Demo`, `Demo Mode/Bindings = Crazy` | Same major actions as crazy bindings on driver controller, with demo speed caps |
| Demo `Code Mode = Demo`, `Demo Mode/Bindings = Competition` | Same driver/operator split as competition bindings, with demo speed caps |

## Autos

- PathPlanner autos come from `AutoBuilder.buildAutoChooser()`.
- Named commands registered in `RobotContainer` include:
  - `Extend Intake`
  - `Start Intake`
  - `Stop Intake`
  - `Intake While Held`
  - `Start Hotdog`
  - `Stop Hotdog`
  - `Start Shoot`
  - `End Shoot`
- `Start Shoot` requests gated forward feed and starts intake. Turret aiming is expected to come from default turret commands.
- If an auto drives correctly but does not shoot, inspect selected auto, path event markers, named-command spelling, and whether indexer/hotdog telemetry changes.
- Existing auto names include `BlueSeedAuto`, `RedSeedAuto`, `PLAYOFF LEFT BATMAN`, `RIGHT GOTHAM DOUBLE`, and `SweepyFerry`.
- Seed autos are path/pose-only and have no events.
- North/south double autos use PathPlanner event markers such as `Intake Zone` and `Shoot Early`; a regression test checks that path event markers carry command payloads.
- PathPlanner event triggers were moved into GUI path markers; do not re-add ad hoc event-trigger logic in `RobotContainer` without a clear reason.

## Important Telemetry Names

- Intake: `Intake/Current Speed`, `Intake/Pivot Position`, `Intake/Pivot Setpoint`, `Intake/Commanded Roller Speed`
- Indexer: `Indexer/Indexer Speed`, `Indexer/Hotdog Speed`, `Indexer/Commanded Indexer Speed`, `Indexer/Commanded Hotdog Speed`, `Indexer/Forward Feed Requested`, `Indexer/Feeding Forward`, `Indexer/Auto Unjam Active`, `Indexer/Hotdog Stall Current Exceeded`
- Drive: `SwerveStates/Measured`, `SwerveStates/Setpoints`, `SwerveChassisSpeeds/Measured`, `SwerveChassisSpeeds/Setpoints`, `Odometry/Trajectory`, `Odometry/TrajectorySetpoint`
- Vision: `Vision/Camera0/RobotPosesAccepted`, `Vision/Camera1/RobotPosesAccepted`, rejected-pose equivalents, and `Vision/Summary/*`
- Turret values are logged through component keys such as `Turret/Left/Azimuth`, `Turret/Right/Flywheels`, plus `@AutoLogOutput` values from `Turret` such as commanded azimuth velocity, fudge factors, and flywheel-ready state.

## Code Gotchas

- `Drive.getMaxLinearSpeedMetersPerSec()` reduces teleop speed while intake or feed is active. In Competition this is fixed at `Constants.SubsystemConstants.Drive.reducedSpeed` (`2.5 m/s` in this checkout). In Demo it comes from `Demo Mode/Reduced Speed While Shooting MPS`.
- Demo mode also caps non-intake/feed drive speed through `Demo Mode/Max Speed MPS`.
- `DriveCommands.joystickDrive(...)` and `joystickDriveAtAngle(...)` have overloads that accept a max-linear-speed supplier. Existing competition call sites delegate to `drive::getMaxLinearSpeedMetersPerSec`; demo bindings pass dashboard-backed suppliers.
- PathPlanner drive/rotation PID is currently `5.0, 0.0, 0.0`.
- Drive code mass/MOI differs from PathPlanner settings; confirm which reflects the real robot before tuning autos.
- `.auto` files report `"version": "2025.0"` in this 2026 project; confirm compatibility before assuming the file format is wrong.
- `GetAdjustedShot.ShootingParameters.isValid()` prints when invalid; a missing or bad shot table can spam stdout.
- Shoot-on-the-move azimuth tracking depends on field-relative chassis speeds, robot-to-turret offsets, TOF values, and tangential acceleration history.
- Forward feed requests are not the same as applied feed; check `Indexer/Forward Feed Requested` and `Indexer/Feeding Forward` separately.
- `Drive.periodic()` stops all modules and clears setpoint logs while disabled.
