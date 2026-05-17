# Action Flows

Use this to understand what should happen when Chuck performs a real robot action.

## Intake FUEL From Carpet

Physical flow:
1. Intake pivots down and forward.
2. Top roller spins to pull FUEL over the kicker bar.
3. FUEL enters the hopper/hotdog floor.

Code flow:
1. Operator/driver trigger schedules `IntakeCommand`.
2. `initialize()` calls `intake.extend()`.
3. `initialize()` calls `intake.setIntakeSpeed(mintakeSpeed + 3 * driveSpeedMetersPerSecond)`.
4. `end()` calls `intake.stopIntake()`.

Important caveat: `IntakeCommand` does not start the hotdogs or indexer.

## Move FUEL Toward Turrets

Physical flow:
1. Hotdogs roll the hopper floor toward the indexer.
2. Indexer feeds the shared turret path.
3. FUEL reaches both turret feed locations.

Code flow:
1. `Indexer.startHotdog()` commands hotdog velocity.
2. `Indexer.startIndexer()` commands indexer velocity.
3. `Indexer.feedCommand()` runs both until interrupted.
4. `RobotContainer` and `CrazyModeBindings` bind shooting/feed triggers to start/stop both.

No current real sensor tells the code where FUEL stopped.

## Shoot At Hub

Physical flow:
1. Turret azimuth turns each shooter toward target.
2. Hood sets launch angle.
3. Flywheels spin to shot speed.
4. Indexer/hotdogs/intake feed FUEL into both turrets.

Code flow:
1. Default `TurretTargeting` reads `drive.getPose()` and `drive.getFieldRelativeSpeeds()`.
2. `GetAdjustedShot` calculates turret-to-target distance and compensates for robot velocity.
3. `shot_table.json` maps distance to hood angle, flywheel RPS, and FUEL time-of-flight.
4. `Turret.runAutoTarget()` sends azimuth, hood, and flywheel setpoints.
5. Shooting/feed commands start indexer, hotdogs, and intake.

The current code does not appear to gate feeding on "turrets ready."

## Shoot While Moving

`GetAdjustedShot` looks ahead by FUEL time-of-flight plus extra latency. It shifts the turret's predicted field position by field-relative velocity, recalculates distance, and aims at the target from that predicted position.

If shots are wrong only while strafing, inspect:

- field-relative velocity from drive
- robot-to-turret offsets
- shot table time-of-flight values
- Limelight/odometry pose stability
- measured flywheel speed under drive load

## Neutral-Zone Ferry Targeting

When the robot is in the neutral zone, a trigger schedules both turrets to target the closer ferry target instead of the hub. Target points come from `Constants.FieldConstants.getCloserFerryTarget()` and are alliance-flipped.

If a turret aims at an unexpected place only in the neutral zone, inspect neutral-zone X bounds, alliance flipping, ferry target constants, and whether the trigger is active.

## Vision Pose Updates

Physical flow:
1. `limelight-back` and `limelight-right` observe AprilTags.
2. Vision estimates robot pose.
3. Drive fuses accepted observations into swerve pose estimation.

Code flow:
1. Real mode constructs two `VisionIOLimelight` instances.
2. `Vision.periodic()` logs each camera, rejects some impossible observations, and computes standard deviations.
3. Accepted observations call `drive.addVisionMeasurement()`.
4. `Drive` fuses them in `SwerveDrivePoseEstimator`.

If pose jumps, compare accepted and rejected poses by camera before changing filters.

## Autonomous

Physical flow:
1. PathPlanner drives the swerve along selected auto paths.
2. Event markers run named robot actions.
3. Turrets keep aiming through their default commands unless interrupted.

Code flow:
1. `Drive` configures `AutoBuilder`.
2. `RobotContainer` registers `NamedCommands`.
3. `autoChooser` uses `AutoBuilder.buildAutoChooser()`.
4. Path files and auto files under deploy define path sequence and events.

If path following works but robot action does not, inspect selected auto, marker names, registered named commands, and corresponding subsystem telemetry.

## Disabled Turret Zeroing

While disabled, each `Turret.periodic()` polls its azimuth zero switch. On the rising edge, it calls `azimuth.zeroPosition()`. During enabled operation, the edge memory is reset and zeroing does not occur.

For a turret that hits a limit, first verify the disabled zero switch behavior and measured/commanded azimuth before widening soft limits.
