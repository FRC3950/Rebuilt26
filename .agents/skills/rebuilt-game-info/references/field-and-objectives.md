# Field And Objectives

This reference is the field-layout mental map for `REBUILT`. Use it when Codex needs to understand where the important objects and zones sit relative to each other before reasoning about autos, pathing, intake locations, shot locations, or climb approach behavior.

## Field Skeleton

- The carpeted field is about `8.07 m` wide by `16.54 m` long.
- The welded-perimeter drawing reference dimensions are `651.22 in` long by `317.688 in`
  wide. The AprilTag sheet defines the origin at the blue alliance station side, with
  `+X` toward the red alliance station, `+Y` away from the scoring table, and `+Z`
  upward from the carpet.
- The long sides are guardrails.
- Each short end is an alliance end bounded by that alliance's `ALLIANCE WALL`, `OUTPOST`, and `TOWER WALL`.
- The field is mirrored left-to-right between alliances.

## Major Field Objects

- `HUB`: primary scoring structure for `FUEL`
- `TOWER`: endgame climbing structure near an alliance end
- `OUTPOST`: alliance-side structure associated with human-player interaction
- `DEPOT`: field-side source location for collecting `FUEL`
- `BUMPS`: raised terrain features that constrain approach paths
- `TRENCHES`: lane-defining field features with low ground hardware and a raised solid bar

## Mental Map

Think of the field as three bands from one alliance wall to the other:

1. An alliance-end band containing that alliance's protected scoring and climb side.
2. A large middle band containing both `HUBS`, the `CENTER LINE`, and the main loose-`FUEL` traffic area.
3. The mirrored alliance-end band for the other alliance.

Inside each alliance-end side:

- The `ALLIANCE AREA` is the shallow rectangle directly in front of the alliance wall.
- The `OUTPOST AREA` sits at the alliance end near the `OUTPOST`, adjacent to the alliance side boundary.
- The `ALLIANCE ZONE` extends farther into the field than the `ALLIANCE AREA` and contains that alliance's `TOWER` and one `DEPOT`.
- The `ROBOT STARTING LINE` is the front boundary of the `ALLIANCE ZONE`, toward midfield.

Between the two alliance zones:

- The `NEUTRAL ZONE` spans the middle of the field.
- The `CENTER LINE` bisects the `NEUTRAL ZONE` across the field width.
- Both `HUBS` sit in this middle region, one for each alliance.
- `BUMPS` and `TRENCHES` help define this central traffic and routing space.

## Relative Positions That Matter

- Each alliance has exactly one `HUB`, one `TOWER`, one `OUTPOST`, and access to one nearby `DEPOT` in its side of the field.
- Each `HUB` is centered between two `BUMPS`.
- Each `HUB` sits `158.6 in` (`~4.03 m`) from its alliance wall, so it is forward of the deepest alliance-end structures and close to the transition into midfield.
- Each `HUB` faces the `NEUTRAL ZONE`; processed `FUEL` exits the base of the hub back into the neutral area.
- The hub assembly drawing gives a `47.00 in` base width and `72.000 +/- .500 in`
  vertical height. Its AprilTags are at `44.25 in` Z.
- The `ALLIANCE ZONE` surrounds the alliance's `TOWER` and `DEPOT`, meaning climbing and depot-side collection happen in the same general end-of-field region.
- The `NEUTRAL ZONE` is bounded by `BUMPS`, `TRENCHES`, `HUBS`, and guardrails, so midfield traffic is naturally structured by these obstacles rather than being one open rectangle.
- The `ROBOT STARTING LINE` is in front of the alliance `HUB` and two `BUMPS`, making autonomous starting geometry immediately relevant to early shots and early traversal choices.

## Drawing-Derived Field Element Geometry

- `BUMP` (`GE-26100`) is a triangular terrain profile. The field drawing gives a
  `48.93 in` bounding-triangle length, `24.47 in` from one end to the virtual peak,
  `44.40 in` width, `6.56 in` virtual peak height, a `15.0 deg` ramp angle, and
  about `0.61 in` edge height. The drawing shows triangular profiles in both the
  length and width directions, so simulation should treat this as traversable peaked
  terrain that changes robot height, pitch, and roll, not as a hard wall.
- `TRENCH` (`GE-26200`) is not a floor-to-ceiling obstacle. It is assembled from
  `GE-26201 Trench Hinged Assy` and `GE-26202 Trench Fixed Assy`. The controlled
  welded-perimeter dimensions show hinged trenches measured closed and locked,
  `50.34 +2.00/-2.00 in` from side reference, `73.00 +1.00/-1.00 in` spans, and
  `22.25 +/- .25 in` to the raised solid bar. Trench AprilTags are at `35.00 in` Z.
  Robot collision/pathing should leave the under-bar lane traversable unless a
  separate robot-height model proves a specific robot cannot clear it.

## Named Areas And Lines

- `ALLIANCE AREA`: the shallow end-of-field area against the alliance wall.
- `ALLIANCE ZONE`: the deeper alliance-side volume that includes the `TOWER` and `DEPOT` and reaches to the `ROBOT STARTING LINE`.
- `OUTPOST AREA`: a marked sub-area near the `OUTPOST`.
- `NEUTRAL ZONE`: the central shared zone between alliance sides.
- `CENTER LINE`: the white line that splits the `NEUTRAL ZONE` in half.
- `HUMAN STARTING LINE`: the white line in the alliance-end area associated with human staging.
- `ROBOT STARTING LINE`: the alliance-colored line at the front of the alliance zone.

## Gameplay Objectives Mapped To Layout

- Collect `FUEL` from the carpet in midfield, from `DEPOTS`, and from human-player related areas near the `OUTPOST`.
- Route from alliance-end staging through or around `BUMPS` and `TRENCHES` to reach favorable scoring positions.
- Score into the `HUB` when it is active, then either continue cycling or rotate toward collection and setup when it is inactive.
- Leave enough time and path access to transition from cycle play back to the alliance-side `TOWER`.

## Robot-Code Implications

- Odometry, auto pathing, and driver aids should treat `HUB`, `TOWER`, `DEPOT`, `CENTER LINE`, and the `ROBOT STARTING LINE` as first-class landmarks.
- Traversal logic should account for the fact that `BUMPS` and `TRENCHES` shape which approaches are smooth, fast, or repeatable.
- Shot routines should understand that the `HUB` lives near the alliance-to-midfield transition rather than deep against the wall.
- Climb routines should assume the robot must return to the alliance-side `TOWER` region from whatever cycle location it currently occupies.

## Scope Note

This file is meant to make the field comprehensible. For exact dimensions, tolerances, or build geometry, consult the official field drawings and CAD.
