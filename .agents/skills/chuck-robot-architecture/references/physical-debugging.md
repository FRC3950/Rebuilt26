# Physical Debugging Map

Use this when a student describes a real symptom. Start with evidence, not edits.

## Standard Questions

Ask or infer only what is needed:

- Which mechanism moved, failed to move, sounded wrong, or jammed?
- Did it happen disabled, auto, teleop, after switching code mode or demo binding mode, while intaking, while shooting, or while driving?
- Is it one side only, one turret only, one camera only, or whole robot?
- Did the command/telemetry show the robot was trying to do the action?
- Was the mechanism unloaded, loaded with FUEL, or under defense/contact?

## Cause Buckets

| Bucket | Typical signs | First checks |
|---|---|---|
| Mechanical | jam, slip, binding, physical stop, FUEL stuck despite motors spinning | inspect mechanism, compare commanded vs measured speed/current, test low power |
| Electrical | device missing, motor does nothing, current spike, intermittent CAN | check CAN bus, CAN ID, power, breaker, motor controller faults |
| Sensor | wrong zero, pose jump, bad switch, gyro/camera disconnect | inspect sensor telemetry, compare redundant sides/cameras, disabled zero behavior |
| Tuning | overshoot, oscillation, slow response, short/long shots | compare setpoint vs measured, review PID/feedforward/shot table |
| Command logic | works from one button/auto but not another | inspect code mode, demo binding mode, bindings, default commands, requirements, named commands |
| Constants/config | wrong side, wrong limit, wrong direction, wrong transform | inspect CAN IDs, inversion, soft limits, gear ratios, robot-to-camera/turret transforms, alliance flip |

## Symptom Lookup

| Physical symptom | Likely code area | Telemetry to inspect | Safe first diagnostic |
|---|---|---|---|
| Left turret hits a limit, right turret aims fine | `Constants.SubsystemConstants.Turret`, `Turret`, `AzimuthIOTalonFX`, `RobotContainer.createRealLeftTurret()` | left/right azimuth inputs, commanded azimuth, measured azimuth, `zeroSwitchClosed` | disabled zero-switch edge test, compare S1/S2 CANdi inputs, do not widen limits first |
| Both turrets aim wrong after enable | disabled zeroing, target pose, alliance flip, turret offsets | commanded/measured azimuth, drive pose, alliance-flipped hub target | disable, re-zero, compare robot pose to field, verify alliance |
| Hood moves wrong direction on one turret | `HoodIOServoHub`, servo channel inversion, physical linkage | hood setpoint and pulse width | command min/mid/max hood angles with robot safe and disabled/low-risk |
| Flywheel spins but FUEL launches weak | `FlywheelsIOTalonFX`, flywheel config, shot table, feed timing | commanded/measured flywheel RPS, current, hood angle | check measured RPS reaches setpoint before feeding |
| Feed trigger is held but FUEL does not move into turrets | `Indexer`, `CompBindings`, `CrazyBindings`, `DemoContainer`, `Turret.isFlywheelReadyForFeed()` | `Indexer/Forward Feed Requested`, `Indexer/Feeding Forward`, commanded/measured flywheel RPS, applied code/binding mode | confirm both turret flywheels are within `2.5 RPS`; reverse/unjam separately to rule out a jam |
| Robot shoots short only while driving sideways | `GetAdjustedShot`, `TurretTargeting`, drive field-relative speeds, `shot_table.json` TOF | `SwerveChassisSpeeds/Measured`, commanded/measured azimuth velocity, hood/flywheel setpoints, measured flywheel RPS, pose | compare stationary vs slow strafe shots and commanded setpoints |
| Intake deploys and spins but FUEL does not reach turrets | `IntakeCommand`, `Intake`, `Indexer`, `IndexerIOTalonFX`, feed bindings | intake speed, pivot position, indexer/hotdog commanded and measured speeds | confirm hotdogs/indexer are commanded; inspect transfer path for jams |
| Hotdogs run backward or fight the indexer | `Indexer`, `IndexerIOTalonFX`, indexer constants/inversion | commanded and measured hotdog/indexer speeds | low-speed test each motor separately, verify physical direction before changing inversion |
| Intake pivots but roller does not spin | `Intake`, `IntakeIOTalonFX`, intake constants | roller connected, commanded roller speed, measured roller speed/current | test roller unloaded, check CAN ID and wiring |
| Intake roller spins but pivot does not deploy | `IntakeIOTalonFX`, pivot Motion Magic config, pivot position constants | pivot connected, pivot setpoint, pivot position/current | test pivot unloaded and confirm zero position before increasing gains |
| Auto follows path but shooting does not happen | PathPlanner `.auto`/`.path`, `NamedCommands`, `RobotContainer`, `Indexer` | auto selection, trajectory logs, indexer/hotdog/intake commands | verify selected auto and marker spelling, then check whether feed outputs ever command |
| Auto shoots but misses the hub | `GetAdjustedShot`, `shot_table.json`, `Drive`, `Vision`, `AllianceFlipUtil` | pose, target pose, hood/flywheel/azimuth setpoints, measured flywheel RPS | compare auto pose to field and test same shot in teleop from same spot |
| Vision pose jumps near the hub | `Vision`, `VisionIOLimelight`, `VisionConstants`, `Drive.addVisionMeasurement()` | accepted/rejected poses per camera, tag IDs, odometry pose | identify whether one camera causes jumps before changing global filters |
| Only right-side camera poses look wrong | `VisionConstants`, Limelight physical config, AdvantageScope robot config | camera 1 accepted poses, tag IDs, camera transform assumptions | compare Java transform to physical mount; known right-camera config mismatch exists |
| Robot drives fine but field-relative feels rotated | gyro IO, pose reset, `DriveCommands`, alliance or heading reset bindings | gyro yaw, robot pose rotation, driver command values | reset heading intentionally and compare robot-relative vs field-relative drive |
| Robot gets slow only while intaking or shooting in Competition | `Drive.getMaxLinearSpeedMetersPerSec`, `Constants.SubsystemConstants.Drive.reducedSpeed`, `Intake.isIntaking`, `Indexer.isFeedingForward` | intake state, indexer feeding state, commanded chassis speeds, `Code Mode/Applied` | confirm the fixed competition reduction is intentional before retuning drive |
| Robot gets too slow or too fast in Demo | `DemoContainer`, `Drive.getMaxLinearSpeedMetersPerSec`, `DriveCommands`, dashboard speed keys | `Code Mode/Applied`, `Demo Mode/Bindings Applied`, `Demo Mode/Max Speed MPS`, `Demo Mode/Reduced Speed While Shooting MPS`, commanded chassis speeds | verify Demo mode is applied while disabled, then adjust dashboard caps instead of changing constants |
| One swerve module behaves differently | `Module`, `ModuleIOTalonFX`, `TunerConstants`, generated configs | measured/setpoint module states, drive/turn currents | raise robot safely, command low speed, compare module direction and encoder readings |
| Button works on one controller/mode but not another | `RobotContainer`, `CompBindings`, `CrazyBindings`, `DemoContainer`, code-mode choosers | `Code Mode/Selected`, `Code Mode/Applied`, `Demo Mode/Bindings Selected`, `Demo Mode/Bindings Applied`, command scheduling, subsystem command state | confirm Competition vs Demo and Demo/Crazy vs Demo/Competition; all mode changes apply only while disabled |
| Auto event marker appears in GUI but robot does not act | PathPlanner marker command payload, registered `NamedCommands`, marker spelling | selected auto, marker names, intake/indexer command telemetry | verify marker has a command payload and matches a registered name |

## Response Pattern

When answering a physical symptom:

1. Name the likely physical section of Chuck.
2. Say what code owns that section.
3. Say what telemetry would prove whether software commanded the action.
4. Give one or two safe tests.
5. Only then mention likely code/config fixes.

If logs are provided, use `frc-assistant` or `frc-reference` telemetry workflow and cite field names, timestamps, and values.
