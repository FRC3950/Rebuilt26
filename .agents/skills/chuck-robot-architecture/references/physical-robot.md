# Physical Robot Reference

Use this when a student describes what Chuck physically did or when code needs robot-shape context.

## Identity And Frame

- Robot name: `Chuck Norris`, Team 3950's 2026 REBUILT robot.
- Primary archetype: double-turret cycle shooter.
- Robot-frame convention for explanations: intake is the front; turrets are toward the rear.
- Game piece: `FUEL`, a foam ball.
- Main scoring structure: `HUB`; use `rebuilt-game-info` for field and scoring context.

## Mechanism Map

| Team term | Physical meaning | Code meaning |
|---|---|---|
| Chuck | Full robot | Whole repo / `RobotContainer` composition |
| Turrets | Two rear shooter assemblies | `Turret` objects built from azimuth, hood, and flywheels |
| Azimuth | Turret rotation axis | `Azimuth`, `AzimuthIOTalonFX`, Motion Magic voltage |
| Hood | Servo-adjusted launch angle | `Hood`, `HoodIOServoHub` |
| Flywheels | Shooter wheels with brass inertia wheel and Colson contact wheels | `Flywheels`, `FlywheelsIOTalonFX`, velocity control |
| Indexer | Ball path feeding both turrets | `Indexer` indexer motor |
| Hotdogs | Rolling hopper floor, sloped toward indexer, with center divider | `Indexer` hotdog motor |
| Intake | 4-bar floor intake with powered top roller and passive kicker bar | `Intake`, `IntakeCommand`, pivot and roller IO |

## Turrets

- Chuck has two turrets, each with a weighted SDS brass flywheel and Colson launch wheels.
- Each turret has two Kraken X44 flywheel motors: one leader and one follower, used to reduce spin-up time and improve maximum speed.
- Each turret hood is moved by one REV Robotics servo.
- Both hood servos share one REV Servo Hub; the Servo Hub is on the roboRIO CAN bus.
- Each turret rotates with one Kraken X44 azimuth motor.
- Azimuth gearing is `10` motor rotations per `1` turret rotation.
- Each azimuth zero position is detected by a limit switch connected through a CTRE CANdi.
- The current code zeroes turret azimuth from the switch while disabled.

## Intake, Hopper, And Feed

- The intake is a 4-bar mechanism that deploys forward/down to pick FUEL from the carpet.
- The intake has a powered top roller and a lower kicker bar that folds out mechanically as the intake deploys.
- FUEL moves from the intake into the hopper without intentional "trash compactor" compression.
- The hotdogs are the moving, sloped hopper floor. They help move FUEL toward the indexer and are divided in the middle to reduce turret-feed jams.
- The indexer is the final powered ball path that feeds both turrets.
- Current code does not expose real beam breaks, time-of-flight sensors, or other FUEL occupancy sensors.

## Drive, Vision, And Electronics

- Drivebase: four SDS MK4i swerve modules with Kraken X60 motors and Colson wheels.
- The code is based on the AdvantageKit CTRE swerve template, upgraded to the IO-style subsystem pattern.
- Chuck uses a roboRIO 2.0, REV PDH, mini PDH, CTRE CANivore, and REV Servo Hub.
- Most CAN devices are on the CTRE CANivore.
- The REV Servo Hub is on the roboRIO CAN bus.
- Vision has two Limelights:
  - `limelight-back`, mounted near the turrets
  - `limelight-right`, mounted on the robot's right side

## Known Physical Context Limits

- No climber is currently described as implemented in this codebase.
- No real FUEL occupancy sensors are currently represented in intake/indexer code.
- If a symptom depends on exact legality, scoring value, or official dimensions, consult the official REBUILT manual rather than this skill.
