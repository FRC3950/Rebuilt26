# Code Architecture Reference

Use this to map robot behavior to code paths before editing.

## Project Shape

- Java WPILib command-based robot.
- GradleRIO 2026 project with AdvantageKit logging.
- CTRE Phoenix 6, REVLib, PathPlannerLib, AdvantageKit, and PhotonVision vendordeps are present.
- `Robot.java` stays thin; `RobotContainer` constructs subsystems, controls, autos, and named commands.
- Hardware code follows an IO pattern:
  - subsystem owns behavior and logging
  - `*IO` defines hardware interface
  - `*IOTalonFX`, `*IOServoHub`, or `*IOLimelight` talks to real hardware
  - `*IOSim` or no-op IO supports sim/replay

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
| Fuel simulation | `sim/FuelSimulationController.java`, `FuelSimCommand.java`, `FuelLaunchCalculator.java` |

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

## Important Telemetry Names

- Intake: `Intake/Current Speed`, `Intake/Pivot Position`, `Intake/Pivot Setpoint`, `Intake/Commanded Roller Speed`
- Indexer: `Indexer/Indexer Speed`, `Indexer/Hotdog Speed`, `Indexer/Commanded Indexer Speed`, `Indexer/Commanded Hotdog Speed`, `Indexer/Feeding Forward`
- Drive: `SwerveStates/Measured`, `SwerveStates/Setpoints`, `SwerveChassisSpeeds/Measured`, `SwerveChassisSpeeds/Setpoints`, `Odometry/Trajectory`, `Odometry/TrajectorySetpoint`
- Vision: `Vision/Camera0/RobotPosesAccepted`, `Vision/Camera1/RobotPosesAccepted`, rejected-pose equivalents, and `Vision/Summary/*`
- Turret values are logged through component keys such as `Turret/Left/Azimuth`, `Turret/Right/Flywheels`, plus `@AutoLogOutput` values from `Turret`.
