# Shot table analysis (throwaway)

Sim-only analysis used to fix `src/main/deploy/shot_table.json`. Nothing here is used by robot code.
Safe to delete this folder and `src/test/java/frc/robot/sim/shottableanalysis/` when done.

## Frozen baseline
`frozen_baseline.json` is the hoodDeg -> launch angle and rps -> exit speed mapping that
`FuelLaunchCalculator.LaunchCalibration.fromShotTable()` derived from the **unedited** table
(`shot_table.original.json`, sha256 in the file). All sweeps use it unless `STA_CAL=derived`.

## Running (WPILib JDK 17; spotless fails under JDK 21)
```
export JAVA_HOME=~/wpilib/2026/jdk
SHOT_TABLE_ANALYSIS=sweep STA_TABLE=<table.json> STA_OUT=<name>.csv ./gradlew test --rerun --tests 'frc.robot.sim.shottableanalysis.*'
python3 shot_table_analysis/show_map.py shot_table_analysis/<name>.csv 0.05
python3 shot_table_analysis/compare.py shot_table_analysis/sweep_baseline.csv shot_table_analysis/<name>.csv
```
Modes: `export` (re-freeze from the current deploy table and self-check; **overwrites** the frozen
file), `sweep`, `search`, `rows`. `STA_CAL=derived` uses whatever the sim derives from the deploy
table right now. The test is skipped in normal builds unless `SHOT_TABLE_ANALYSIS` is set.

## Setup
- Launch point = turret position; robot heading only matters through where the turret ends up.
- Region: turret x in [0.30, 3.75] m, y within 0.30 m of the side walls (blue alliance zone, up to
  the bump/trench line at x = 3.96 and the hub face), distance 1.00-5.70 m in 0.05 m steps,
  bearing -90..+90 deg in 5 deg steps. Distance is to the robot's aim point (`FieldConstants.hubGuy`),
  i.e. the table key.
- Robot pipeline mirrored for a stationary robot: aim at `hubGuy`, table lookup identical to
  `GetAdjustedShot` (checked), hood clamp 13-29.85, rps clamp 0-100.
- One fuel on an empty field, air resistance on, deterministic (the only randomness is the
  post-score dispersal).
- Perturbations (assumed, adjustable in the test): aim +/-1 deg, hood +/-1 deg, rps +/-2.5%.
  `pert27` = 3x3x3 grid at those tolerances; `ext125` = 5x5x5 grid at up to 2x.

## Peak table (`src/main/deploy/shot_table_peak.json`, used when the "Peak Mode" switch is on)
Built by `peak_table.py 3.6` from `search_peak.csv` against the frozen baseline: every row scores at
every in-region bearing with 27/27 perturbed shots, apex held at 3.6 m wherever hood 13 allows
(from 3.25 m out), hood and rps never decrease, ties go to the shot that crosses closest to center.
The 1.3 m row is copied from the patched table (close range can't be made robust).
Maps: `map_peak_frozen.txt` (frozen baseline), `map_peak_derived.txt` (what the live sim derives
from the patched shot_table.json).
