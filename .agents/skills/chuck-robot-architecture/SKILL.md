---
name: chuck-robot-architecture
description: Use when debugging or explaining Team 3950's Chuck Norris robot, especially when a physical symptom, mechanism behavior, REBUILT action, subsystem interaction, auto issue, turret shot, intake/indexer problem, vision pose jump, or AdvantageKit log needs to be mapped to this Java robot codebase.
---

# Chuck Robot Architecture

## Overview

Use this skill to translate real robot behavior into code context. Start from what Chuck physically did, map it to mechanisms and game actions, then inspect the smallest relevant code and telemetry path before proposing changes.

Use `rebuilt-game-info` when the symptom depends on REBUILT field context: hub shots, ferry targets, neutral-zone behavior, autos, alliance flipping, FUEL flow, or field landmarks.

## Physical-To-Code Workflow

1. Restate the symptom in robot terms: mechanism, side, match mode, driver action, and whether it is repeatable.
2. Identify the physical path: intake, hotdogs, indexer, turret, drive, vision, auto, or electronics.
3. Load only the needed reference:
   - `references/physical-robot.md` for mechanism layout, names, frame convention, electronics, and CAN context.
   - `references/code-architecture.md` for subsystem, command, IO, constants, auto, and telemetry structure.
   - `references/action-flows.md` for intake, feed, shoot, targeting, vision, auto, and disabled zeroing flows.
   - `references/physical-debugging.md` for symptom-to-code lookup and diagnostic prompts.
   - `references/current-gaps.md` for intended robot behavior that is missing or partial in code.
4. Inspect the current code before editing. The robot is changing during build season; references explain intent, but files are source of truth for implementation.
5. Separate likely causes into mechanical, electrical, sensor, tuning, command logic, and constants/config before deciding that code is wrong.
6. Recommend safe tests first: disabled checks, low-speed commands, one-mechanism-at-a-time tests, log review, and comparison between left/right or camera 0/camera 1.

## Answer Shape

For physical robot debugging, answer with:

- involved physical parts and team nicknames
- likely code files/classes and constants
- AdvantageKit or NetworkTables keys to inspect
- safe on-robot diagnostics
- the smallest likely code/config change only after evidence points there

Do not jump straight to code edits for symptoms that could be a jam, wiring issue, missing zero, bad camera mount, wrong selected auto, or operator-control mismatch.

## Core Mental Model

Chuck is an intake-front, rear double-turret REBUILT cycle shooter. FUEL enters through the 4-bar intake, moves through the rolling hopper floor called the hotdogs, reaches the indexer, and is fed to two turret shooters. Drive pose comes from CTRE swerve odometry fused with two Limelights. Turret shot setpoints come from robot pose, target position, robot velocity, turret offset, and `shot_table.json`.

The current code has no climber subsystem, no real FUEL occupancy sensors, and no hub-active game-state model. Treat those as known gaps, not things to infer from missing files.
