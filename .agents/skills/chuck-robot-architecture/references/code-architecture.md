# Code Architecture Reference

Use this to map robot behavior to code paths before editing.

## Project Shape

- Java WPILib command-based robot.
- GradleRIO 2026 project with AdvantageKit logging.
- CTRE Phoenix 6, REVLib, PathPlannerLib, AdvantageKit, and PhotonVision vendordeps are present.
- Toolchain snapshot from this checkout: GradleRIO `2026.2.1`, Phoenix 6 `26.1.3`, REVLib `2026.0.5`, PathPlannerLib `2026.1.2`, AdvantageKit `26.0.2`.
- `Robot.java` stays thin; `RobotContainer` constructs subsystems, controls, autos, and named commands.
- Hardware code follows an IO pattern:
  - subsystem owns behavior and logging
  - `*IO` defines hardware interface
  - `*IOTalonFX`, `*IOServoHub`, or `*IOLimelight` talks to real hardware
  - no-op IO supports replay/unavailable hardware paths

## Where To Search First

| Physical area | Code to inspect |
|---|---|
| Robot composition, autos, bindings | `RobotContainer.java` |
| Drivebase, PathPlanner, odometry | `subsystems/drive/Drive.java`, `Module*`, `Gyro*`, `generated/TunerConstants.java` |
| Driver control | `commands/DriveCommands.java`, `controls/CrazyModeBindings.java`, `RobotContainer.configureCompetitionBindings()` |
| Intake deploy/roller | `commands/IntakeCommand.java`, `subsystems/intake/*`, `Constants.SubsystemConstants.Intake` |
| Hotdogs/indexer/feed | `subsystems/indexer/*`, `RobotContainer` named commands and trigger bindings |
| Turret command surface | `subsystems/turret/Turret.java`, `TurretTargeting.java`, `GetAdjustedShot.java` |
| Turret hardware | `turret_base/azimuth/*`, `turret_base/flywheels/*`, `turret_base/hood/*` |
| Shot tuning | `src/main/deploy/shot_table.json`, `util/Distancer.java`, `GetAdjustedShot.java` |
| Vision pose | `subsystems/vision/*`, `VisionConstants.java`, `Drive.addVisionMeasurement()` |
| Field transforms | `Constants.FieldConstants`, `util/AllianceFlipUtil.java`, `util/Zones.java` |
| Autos and event markers | `src/main/deploy/pathplanner/autos`, `src/main/deploy/pathplanner/paths`, `RobotContainer` `NamedCommands` |

## Subsystems

- `Drive`: four-module swerve, pose estimator, PathPlanner `AutoBuilder`, PathPlanner logging, SysId hooks, and vision measurement fusion.
- `Intake`: pivot setpoint, roller velocity command, intake state logging, and simple extend/retract/on/off commands.
- `Indexer`: two controlled outputs: indexer motor and hotdog motor. `isFeedingForward()` is command-state based, not piece-sensor based.
- `Turret`: combines azimuth, hood, and flywheels. It clamps hood angle, selects a safe azimuth wrap within limits, and zeroes azimuth while disabled.
- `Vision`: accepts pose observations from both cameras, rejects some impossible observations, and forwards accepted poses into drive odometry.

## Commands And Controls

- `IntakeCommand` extends the intake and starts the roller. It does not run the indexer or hotdogs.
- Teleop competition bindings use a driver/operator split.
- `CrazyModeBindings` ports major actions to one controller.
- Binding mode can only be applied while disabled.
- Turrets have default `TurretTargeting` commands in competition defaults.
- `Turret.toggleTurretMode()` changes a static targeting mode used by both turrets.
- Competition driver controller is port `0`; operator controller is port `1`.
- Driver controls swerve, heading reset, and hub-facing drive lock.
- Operator controls intake, retract, feed/shoot, unjam/reverse, and turret mode toggle.
- `CRAZY` mode moves major actions to the driver controller only.

## Important Trigger Bindings

| Mode/input | Behavior |
|---|---|
| Driver left stick + right X | Default swerve drive |
| Driver `Y` | Reset current pose rotation to zero while preserving translation |
| Driver `A` held | Drive while facing the hub target |
| Operator left trigger held | `IntakeCommand`: extend intake and run roller |
| Operator right bumper | Retract intake |
| Operator right trigger held | Start indexer and hotdog; stop both on release |
| Operator `B` held | Reverse intake and hotdog for unjam |
| Operator `A` | Toggle shared turret targeting lock mode |
| Robot in neutral zone | Both turrets target closer ferry target instead of hub |

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
- `Start Shoot` starts indexer, hotdogs, and intake. Turret aiming is expected to come from default turret commands.
- If an auto drives correctly but does not shoot, inspect selected auto, path event markers, named-command spelling, and whether indexer/hotdog telemetry changes.
- Existing auto names include `BlueSeedAuto`, `RedSeedAuto`, `PLAYOFF LEFT BATMAN`, `RIGHT GOTHAM DOUBLE`, and `SweepyFerry`.
- Seed autos are path/pose-only and have no events.
- North/south double autos use PathPlanner event markers such as `Intake Zone` and `Shoot Early`; a regression test checks that path event markers carry command payloads.
- PathPlanner event triggers were moved into GUI path markers; do not re-add ad hoc event-trigger logic in `RobotContainer` without a clear reason.

## Important Telemetry Names

- Intake: `Intake/Current Speed`, `Intake/Pivot Position`, `Intake/Pivot Setpoint`, `Intake/Commanded Roller Speed`
- Indexer: `Indexer/Indexer Speed`, `Indexer/Hotdog Speed`, `Indexer/Commanded Indexer Speed`, `Indexer/Commanded Hotdog Speed`, `Indexer/Feeding Forward`
- Drive: `SwerveStates/Measured`, `SwerveStates/Setpoints`, `SwerveChassisSpeeds/Measured`, `SwerveChassisSpeeds/Setpoints`, `Odometry/Trajectory`, `Odometry/TrajectorySetpoint`
- Vision: `Vision/Camera0/RobotPosesAccepted`, `Vision/Camera1/RobotPosesAccepted`, rejected-pose equivalents, and `Vision/Summary/*`
- Turret values are logged through component keys such as `Turret/Left/Azimuth`, `Turret/Right/Flywheels`, plus `@AutoLogOutput` values from `Turret`.

## Code Gotchas

- `Drive.getMaxLinearSpeedMetersPerSec()` reduces teleop speed to `2.5 m/s` while intake or feed is active.
- PathPlanner drive/rotation PID is currently `5.0, 0.0, 0.0`.
- Drive code mass/MOI differs from PathPlanner settings; confirm which reflects the real robot before tuning autos.
- `.auto` files report `"version": "2025.0"` in this 2026 project; confirm compatibility before assuming the file format is wrong.
- `GetAdjustedShot.ShootingParameters.isValid()` prints when invalid; a missing or bad shot table can spam stdout.
- `Drive.periodic()` stops all modules and clears setpoint logs while disabled.
