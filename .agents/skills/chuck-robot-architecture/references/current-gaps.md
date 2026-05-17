# Current Gaps And Intent Notes

Use this to avoid inventing robot capabilities that are not currently in the repo.

## Implemented Or Represented

- CTRE/AdvantageKit swerve drive with PathPlanner auto support.
- Intake pivot and roller control.
- Hotdog and indexer velocity control.
- Two turrets with azimuth, hood, and flywheel control.
- REV Servo Hub hood output.
- CANdi-backed turret zero switch reads.
- Limelight-backed real vision and PhotonVision-backed sim vision.
- Shot table interpolation for hood angle, flywheel RPS, and time of flight.
- Shoot-on-the-move compensation using field-relative velocity.
- Sim support for FUEL intake and launch behavior.

## Known Missing Or Partial

- No climber subsystem is currently present.
- No real FUEL occupancy sensors are currently represented in intake, hotdog, or indexer code.
- No explicit hub-active or hub-inactive game-state model is currently represented.
- No current command appears to gate feeding on turret-ready, flywheel-ready, or hood-ready state.
- `Start Shoot` starts intake, hotdogs, and indexer; it relies on default turret targeting for aim/setpoints.
- Turret azimuth limits are marked as placeholders in constants and should be treated carefully.
- Physical robot intent includes a divided hopper/hotdog floor, but software treats hotdogs as one motor output.
- The code can simulate FUEL capacity and launch behavior, but real robot FUEL position is not directly sensed.

## How To Discuss Gaps

Use language like:

- "The physical robot is intended to..."
- "The current code represents this as..."
- "I do not see a real sensor/code path for..."
- "This is likely a mechanical/electrical diagnosis unless telemetry shows the command is wrong."

Avoid saying a subsystem exists just because REBUILT rewards it. In particular, do not assume a climber.
