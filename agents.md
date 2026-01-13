# GitHub Copilot Instructions – FRC Team 3950 Robot Code

You are assisting **FRC Team 3950** with their **FRC robot code**. The primary goals are:
- Generate **correct, buildable Java robot code** for the current FRC season.
- Follow **WPILib command-based best practices** and the existing project structure.
- Match the **current vendor APIs** (WPILib, Phoenix, REV, PathPlanner, etc.) and **never guess** method signatures.

These instructions apply to all FRC3950 robot repositories, including (but not limited to) `pepperadvanced`, `justinrewrite25`, and any future season repos with similar structure.

---

## 1) Non-negotiables (read this like a preflight checklist)

### 1.1 Safety first
- Never produce code that can run motors “forever” without an obvious stop condition.
- Avoid blocking loops (`while(true)`, long `Thread.sleep`, busy waits). Use Command-based scheduling and timers.
- Always ensure actuators are set to a known safe state when disabled and when commands end/interrupted.
- Prefer conservative defaults (brake/coast, current limits, ramp rates, soft limits) and make them constants.

### 1.2 Don’t guess vendor APIs
- If you are writing or editing code that calls WPILib / CTRE / REV APIs, verify method/class names and expected units by consulting the “Authoritative Docs” section below.
- If the environment cannot access those docs, ask the user to paste the relevant documentation snippet or Javadoc, then proceed.
- If uncertain, explicitly say what is uncertain and propose a safe fallback.

### 1.3 Units & coordinate sanity
- Use meters, radians, seconds wherever possible (WPILib standard).
- Be explicit about coordinate frames (field-relative vs robot-relative) and rotation direction conventions.

### 1.4 Prefer small, reviewable changes
- Keep diffs tight. Avoid “refactors for fun” during build season unless requested.
- When implementing a feature, include a short “what changed / how to test on robot” note in your response.

---

## 2) Tech Stack & Libraries (assumptions to validate in-repo)

Assume the robot code uses:

- **Language**
  - Java (Gradle-based WPILib project)

- **Frameworks / Libraries**
  - **WPILib** (current FRC season – check `build.gradle` and `.wpilib`)
  - **Command-based framework** (`edu.wpi.first.wpilibj2.command.*`)
  - **PathPlanner** (if `.pathplanner` directory exists)
  - **Limelight** (if the team is using vision; commonly via NetworkTables or LimelightHelpers)
  - **Vendor libraries** via `vendordeps`:
    - CTRE **Phoenix 6** (`com.ctre.phoenix6.*`) when present
    - REV **REVLib** (`com.revrobotics.*`) when present
    - Other vendor libs if defined in `vendordeps` / `build.gradle`

---

## 3) 2026 Vendor Baseline (versions + install URLs + compatibility)

### 3.1 Current baseline versions (update when the team upgrades)
- WPILib / GradleRIO: 2026.1.1 (Kickoff release)
- Java: 17 (standard WPILib Java baseline)
- CTRE Phoenix 6: 26.1.0 (FRC 2026 kickoff release)
- REVLib: 2026.0.0
- REV Hardware Client: use REV Hardware Client 2 for REVLib 2026+

If the repo’s `build.gradle` / vendordeps indicate different versions, align instructions and behavior to the repo’s actual versions.

### 3.2 Vendor “vendordep” install URLs (for WPILib → Manage Vendor Libraries)

CTRE Phoenix 6 (FRC 2026):
- Phoenix 6: https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2026-latest.json
- Phoenix 6 Replay: https://maven.ctr-electronics.com/release/com/ctre/phoenix6replay/latest/Phoenix6-REPLAY-frc2026-latest.json

REVLib (2026):
- REVLib: https://software-metadata.revrobotics.com/REVLib-2026.json

Rules for vendordeps:
- Vendordep JSON files live in `/vendordeps`. They should be committed to the repo.
- If you add a new vendor lib, add the vendordep file + update code accordingly. Do not “half install” a library.

### 3.3 Firmware compatibility notes (important)
CTRE Phoenix 6:
- Phoenix 6 26.1.0 requires device firmware 26.1.0+.
- Phoenix 6 26.1.0 is not compatible with 2026 beta firmware.

REV:
- Ensure controllers and devices are updated using the correct REV Hardware Client version for the season (REV Hardware Client 2 for REVLib 2026+).

---

## 4) Authoritative Docs (the “source of truth” list)

When you need to reference API details, prefer these:

### 4.1 WPILib (FRC 2026)
- Docs (stable): https://docs.wpilib.org/en/stable/
- Docs mirror (same content): https://frcdocs.wpi.edu/en/stable/
- WPILib 2026 yearly changes: https://docs.wpilib.org/en/stable/docs/yearly-overview/yearly-changelog.html
- WPILib Java API (release): https://github.wpilib.org/allwpilib/docs/release/java/
- Third-party libraries guide: https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html

### 4.2 CTRE Phoenix 6 (FRC 2026)
- Installation docs (stable): https://v6.docs.ctr-electronics.com/en/stable/docs/installation/installation.html
- API docs (Java, release): https://api.ctr-electronics.com/phoenix6/release/java/
- General docs (stable): https://v6.docs.ctr-electronics.com/en/stable/
- “New for 2026” Phoenix 6 changelog: https://v6.docs.ctr-electronics.com/en/latest/docs/yearly-changes/yearly-changelog.html
- CTRE Phoenix releases (official): https://github.com/CrossTheRoadElec/Phoenix-Releases/releases

### 4.3 REVLib (FRC 2026)
- REVLib docs: https://docs.revrobotics.com/revlib
- REVLib install page (check version / vendordep url): https://docs.revrobotics.com/revlib/software-installation
- REVLib install page (includes current version + vendordep URL): https://docs.revrobotics.com/revlib/home/install
- REV Hardware Client docs: https://docs.revrobotics.com/rev-hardware-client/

### 4.4 PathPlanner (2026)
- PathPlanner docs: https://pathplanner.dev/
- PathPlanner “Getting Started”: https://pathplanner.dev/gui-getting-started.html
- PathPlannerLib install info: https://pathplanner.dev/pplib-getting-started.html

### 4.5 Limelight (Vision)
- Main site: https://limelightvision.io
- Docs home: https://docs.limelightvision.io/
- Limelight NetworkTables API (authoritative key list): https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
- LimelightHelpers / Limelight Lib guidance: https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib
- Robot localization overview: https://docs.limelightvision.io/docs/docs-limelight/pipeline-april-tag/apriltag-robot-localization

Doc-access rule:
- If you cannot access the docs from your current environment/tooling, request the specific page/section content from the user rather than guessing.

---

## 5) WPILib 2026 season notes (practical implications)

- WPILib 2026 requires a 2026 RoboRIO image and the FRC 2026 Game Tools process (drivers/tools must match the season).
- If migrating from a 2025 project, prefer the WPILib “import to 2026” workflow so automated breaking-change fixes get applied, then re-install vendor libraries for 2026 compatibility.

When writing answers, assume the user is operating inside the 2026 toolchain and highlight any mismatch risks.

---

## 6) Repository structure & conventions (Command-based)

Assume standard WPILib Java project layout:

- `src/main/java/frc/robot/Robot.java`
  - Minimal logic: lifecycle hooks, construct RobotContainer, call scheduler.
  - No huge hardware init blobs here.

- `src/main/java/frc/robot/RobotContainer.java`
  - Owns subsystem instances, command instances, controller bindings (Triggers), and auto chooser.

- `src/main/java/frc/robot/subsystems/*`
  - Each subsystem owns its hardware objects + periodic logic.
  - Subsystems expose methods that represent “robot actions” (e.g., `setIntakeVoltage()`, `setPivotAngleRadians()`, `stop()`). 

- `src/main/java/frc/robot/commands/*`
  - Commands orchestrate subsystem actions and define end conditions.
  - Prefer small, composable commands over giant monolith commands.

- `src/main/java/frc/robot/constants/*` (or `Constants.java`)
  - All IDs, gear ratios, PID gains, limits, and tuning numbers.
  - Keep CAN IDs, DIO ports, and controller ports centralized.

Conventions:
- Prefer “constants per mechanism” classes (e.g., `DriveConstants`, `ShooterConstants`) over one mega class.
- Avoid magic numbers in subsystem/command logic—promote them to constants with comments.

---

## 7) Architecture conventions (Command-based WPILib)

Follow command-based best practices:
- Keep `Robot.java` thin (lifecycle + scheduler calls).
- Put bindings, default commands, and auto selection in `RobotContainer`.
- Subsystems own hardware and provide small, intentional methods (e.g., `setVoltage()`, `setTargetAngle()`, `stop()`).
- Commands coordinate subsystems; commands should not “new up” hardware.
- Commands must:
  - be non-blocking (no sleep, no long IO)
  - explicitly stop outputs in `end(interrupted)` when appropriate
  - avoid reconfiguring motor controllers every loop

- Prefer **command-based** robot structure (`edu.wpi.first.wpilibj2.command`) for subsystems and commands.
- Use constants classes for device IDs and tuning values.
- Keep hardware access in subsystems, not in commands.
- Favor example patterns from WPILib and vendor docs over ad-hoc patterns found online.

---

## 8) Coding Style & Readability

Follow a clean, modern Java style consistent with common FRC conventions:

### 8.1 Naming
- Class names: `UpperCamelCase` (`SwerveSubsystem`, `DriveToPoseCommand`).
- Methods: `lowerCamelCase` (`driveFieldRelative`, `getPose`, `resetEncoders`).
- Constants: `UPPER_SNAKE_CASE` (`DRIVE_MOTOR_ID`, `MAX_SPEED_METERS_PER_SECOND`).
- Fields:
  - `private final` for hardware objects and immutable collaborators.
  - Avoid meaningless names like `m1`, `s1`; prefer `frontLeftModule`, `armMotor`, `gyro`.

### 8.2 Structure
- Keep methods short and focused.
- Avoid deeply nested `if`/`else` trees where possible.
- Prefer small helper methods over copy-pasting blocks of logic.

### 8.3 Comments
- Explain *why* something is done, not just *what*.
- For tuning and constants, include a brief note about where values came from (characterization tool, practice field, etc.).

### 8.4 Safety & competition constraints
- Never remove or bypass safety checks without a clear reason.
- Avoid code that can block main loops or starve the command scheduler.
- When in doubt, keep behavior fail-safe (motors stop on errors, invalid states are handled gracefully).

---

## 9) Command-Based Best Practices (how to build commands that survive competition)

When creating or modifying commands:

### 9.1 Use composition over giant “god commands”
- Prefer combining simple commands using `andThen`, `alongWith`, `raceWith`, `deadline` compositions.
- Avoid monolithic commands that try to handle everything (driving, scoring, balancing) in one class.

### 9.2 Default commands
- Every subsystem that needs continuous behavior (e.g., drivetrain, turret tracking) should have a default command that:
  - Reads operator inputs and
  - Issues outputs to the subsystem.
- Make sure to set default commands in `RobotContainer`.

### 9.3 Lifecycle methods
- `initialize()`: reset state, timers, sensors relevant to this command.
- `execute()`: simple, per-tick updates using current sensor and input values.
- `end(interrupted)`: safely stop motors, close loops, or reset states as needed.
- `isFinished()`: return true only when the command is logically complete.

### 9.4 No busy waiting
- Do not use `Thread.sleep`, `while` loops that wait on time, or other blocking patterns.
- Use timers, `Command` compositions, or `WaitCommand` instead.

---

## 10) Command-based best practices (FRC reality-proof)

1) Avoid blocking code
- Don’t block in `periodic()`.
- Don’t use `Timer.delay()` in robot code (except extremely rare test-only situations).

2) Always define command end conditions
- For motion commands: end on tolerance + timeout.
- For “run motor” commands: require a trigger held, or require explicit cancel.

3) Use Triggers cleanly
- Prefer `Trigger` / `CommandXboxController` patterns.
- Keep bindings readable and centralized in RobotContainer.

4) Default commands
- Each “always active” subsystem (drive, intake rollers, etc.) should have a sane default command.
- Defaults should be safe at zero input and should not fight with other commands.

5) Interruption behavior matters
- Implement `end(boolean interrupted)` carefully: stop motors, hold position, or return to neutral safely.

---

## 11) Drivetrain / Swerve Guidance (when applicable)

If the project uses a **swerve drive** (common for FRC3950):

- Keep all swerve-specific logic inside a dedicated subsystem (e.g., `SwerveSubsystem`).
- Use WPILib `SwerveDriveKinematics`, `SwerveDriveOdometry`, and units classes when present.
- Drive API design:
  - Provide field-relative and robot-relative drive helpers, for example:
    - `drive(Translation2d translation, double rotation, boolean fieldRelative, boolean openLoop)`
  - Keep the teleop drive command thin, delegating kinematics and conversions to the subsystem.

- Constants:
  - CONTAIN:
    - `TRACK_WIDTH_METERS`
    - `WHEEL_BASE_METERS`
    - `DRIVE_GEAR_RATIO`
    - `ANGLE_GEAR_RATIO`
    - `MAX_SPEED_METERS_PER_SECOND`
    - Gyro inversion flags and offsets
  - Use these constants consistently when configuring modules and odometry.

- Tuning:
  - If you generate tuning code (PID, feedforward), prefer:
    - Reasonable default gains
    - Clear comments reminding students to tune on-robot
    - Non-aggressive defaults to reduce oscillation risk

---

## 12) Motor controller & sensor guidelines (vendor-agnostic + vendor-specific)

### 12.1 Universal motor safety checklist
Whenever creating/configuring a motor controller:
- Set neutral behavior (brake/coast) intentionally.
- Set current limits intentionally.
- Set ramp rates intentionally (especially drivetrains).
- Set inversion intentionally and document why.
- Define soft limits when mechanisms can crash into themselves.
- Provide an “emergency stop” method (often just `stop()`).

### 12.2 CTRE Phoenix 6 guidance
When writing CTRE code:
- Prefer Phoenix 6 APIs and patterns (do not mix with Phoenix 5 unless the repo already does).
- Use Phoenix 6 APIs consistent with the 2026 kickoff release.
- Use the CTRE docs for:
  - config objects,
  - control requests,
  - signal frequency and status frame management,
  - firmware/version requirements.
- Keep configuration objects in constants, apply configuration during subsystem construction/init.
- Do not spam/re-apply configuration calls in periodic loops.
- Be explicit about CAN bus name if using secondary CANivore networks.
- Be careful with units (Phoenix 6 frequently uses typed units or specific scaling); confirm expected units from CTRE docs/Javadocs.

Common pitfalls to avoid:
- Re-applying configuration every periodic loop.
- Forgetting to apply current limits / voltage compensation / neutral mode.
- Using wrong units (Phoenix 6 frequently uses typed units or specific scaling).

If the repo uses CTRE Tuner-generated configs (e.g., swerve), treat those JSON/config artifacts as authoritative and avoid rewriting them unless requested.

Firmware note: if behavior seems “weird” or devices don’t respond, suspect firmware/library mismatch first.

### 12.3 REVLib guidance
When writing REV code:
- Use REVLib APIs consistent with the 2026 season.
- Confirm the exact controller type used (SPARK MAX vs SPARK Flex) and the sensor type (internal encoder vs alternate).
- Apply configuration once (idle mode, current limits, conversion factors).
- Configure current limits, idle mode, conversion factors, PID, and soft limits centrally.
- For closed-loop:
  - use the correct REV API objects (PID controller objects, encoders),
  - keep gains in constants,
  - add telemetry for debugging.
- Avoid “burn flash” / persistent config calls unless you know the team’s policy; if unsure, ask.
- Always confirm method names and behavior from REVLib 2026 docs when uncertain.

Tooling note: REV firmware/config workflows depend on the correct REV Hardware Client version for the season.

### 12.4 PathPlanner Guidance (2026)
PathPlanner is used for autonomous trajectories + event-driven actions.

Rules:
1) **Do not guess PathPlannerLib API.** Confirm using PathPlanner docs and/or the vendordep Javadocs for your installed version.
2) Put Auto configuration in one place (often in the drive subsystem init or a dedicated auto config class).
3) Autos should be discoverable:
   - provide a `SendableChooser<Command>` for Driver Station selection,
   - name autos clearly.

Version note:
- If the team uses Choreo and loads its trajectory JSON files, ensure PathPlannerLib >= 2026.1.2 (fixes a known v2026.1.1 incompatibility with Choreo JSON version 3).

Operational best practices:
- Keep `.pathplanner/` project files committed.
- Keep path names stable so auto selection doesn’t break.
- Prefer event markers / named commands over hardcoding time-based “wait then do X”.

### 12.5 Limelight Guidance (2026)
Limelight integration has two common patterns:
1) **Direct NetworkTables reads/writes** (pure WPILib approach), or
2) **LimelightHelpers** convenience wrapper (Limelight-provided helper).

Rules:
- The NetworkTables key list changes over time; always use the Limelight docs “Complete NetworkTables API” as the source of truth.
- Typical targeting signals are things like `tv`, `tx`, `ty`, `ta`, but confirm the exact keys and coordinate frames from the docs before coding.

Pose estimation best practices:
- If fusing Limelight pose into WPILib odometry:
  - use the appropriate Limelight pose entry for your alliance/origin convention (WPILib origin matters),
  - compensate for pipeline + capture latency when timestamping vision measurements,
  - gate vision updates (reject clearly bad measurements, e.g., no targets, huge jumps, low confidence).

Pipeline/LED control:
- Only write NT control values (pipeline/LED/camMode) from one location in code to avoid fighting behaviors.

If asked to implement Limelight integration, prefer creating a small `VisionSubsystem` that:
- reads Limelight data,
- publishes diagnostics (target valid, latency, pose),
- provides `Optional`-style accessors for consumer code.

---

## 13) Telemetry, debugging, and “don’t-fly-blind” practices

- Publish high-value telemetry:
  - mechanism setpoints vs measured values
  - motor currents / temperatures
  - sensor health / faults
  - command state (what command is currently running)

- Prefer structured naming for NetworkTables keys (stable, discoverable).
- If using logs (WPILib datalog or other), keep keys stable across events so AdvantageScope layouts don’t break.

If you introduce a new subsystem, include:
- a minimal telemetry set
- a simple “smoke test” command (manual run at safe power, with timeout)

---

## 14) Build / test / deploy expectations (what Copilot should assume)

When making code changes, aim for:
- Code compiles (`./gradlew build`)
- Tests (if present) pass (`./gradlew test`)
- If simulation is used, ensure sim still runs (`./gradlew simulateJava` or project-specific sim task)

If asked to provide instructions, default to these GradleRIO patterns unless the repo differs.

---

## 15) “Always check docs” — how to operationalize it

When asked to implement something involving WPILib / Phoenix 6 / REVLib:

1) Identify the exact classes/methods involved.
2) Cross-check the authoritative docs listed above.
3) Implement using the verified API.
4) If docs can’t be accessed:
   - Ask for a snippet (or a link the user can open locally),
   - OR propose two candidate implementations clearly labeled,
   - AND recommend the safest fallback (usually “do nothing dangerous”).

---

## 16) Output format expectations (how Copilot should respond)

When you propose code changes, include:
- A brief plan (files to change + why).
- The implementation.
- How to test:
  - in sim (if applicable)
  - on robot (step-by-step, safe power first)
- Any assumptions made.

When Copilot produces code, it must include:
- correct package + imports
- consistent formatting
- comments only where they add meaning (not noise)
- TODO markers where hardware IDs/configs are unknown
- a short “how to test this” note when adding non-trivial behavior (sim steps or driver station steps)

If compilation is likely to fail due to API uncertainty, do NOT guess. Provide two parts:
1) a safe skeleton,
2) exactly what needs to be confirmed in docs (with the doc URL).

Keep answers practical and “student-usable.”
