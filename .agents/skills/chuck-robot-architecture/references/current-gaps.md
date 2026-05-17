# Current Gaps And Intent Notes

Use this to avoid inventing robot capabilities that are not currently in the repo.

## Implemented Or Represented

- CTRE/AdvantageKit swerve drive with PathPlanner auto support.
- Intake pivot and roller control.
- Hotdog and indexer velocity control.
- Two turrets with azimuth, hood, and flywheel control.
- REV Servo Hub hood output.
- CANdi-backed turret zero switch reads.
- Limelight-backed real vision.
- Shot table interpolation for hood angle, flywheel RPS, and time of flight.
- Shoot-on-the-move compensation using field-relative velocity.

## Known Missing Or Partial

- No climber subsystem is currently present.
- No real FUEL occupancy sensors are currently represented in intake, hotdog, or indexer code.
- No explicit hub-active or hub-inactive game-state model is currently represented.
- No current command appears to gate feeding on turret-ready, flywheel-ready, or hood-ready state.
- `Start Shoot` starts intake, hotdogs, and indexer; it relies on default turret targeting for aim/setpoints.
- Turret azimuth limits are marked as placeholders in constants and should be treated carefully.
- Intake pivot positions are also marked as placeholder/update-me values.
- Hood position has no feedback sensor in code; the stored hood position is the commanded pulse-derived value.
- Hood servos share one static Servo Hub and are configured to keep power while disabled.
- `Turret.lockedIn` is static, so the lock-mode toggle affects both turret instances.
- Physical robot intent includes a divided hopper/hotdog floor, but software treats hotdogs as one motor output.
- Real robot FUEL position is not directly sensed.
- `limelight-right` has a known transform mismatch between Java constants and AdvantageScope config; confirm physical authority before editing.
- PathPlanner settings do not exactly match drive-code mass/MOI/module spacing; confirm before tuning auto tracking.

## How To Discuss Gaps

Use language like:

- "The physical robot is intended to..."
- "The current code represents this as..."
- "I do not see a real sensor/code path for..."
- "This is likely a mechanical/electrical diagnosis unless telemetry shows the command is wrong."

Avoid saying a subsystem exists just because REBUILT rewards it. In particular, do not assume a climber.
