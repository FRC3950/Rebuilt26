# Chuck Norris Shot Lab — complete measurement and tuning guide

A field guide for the version-2 radial quadratic model on branch `chuck-shot-lab`. Prepared from the current implementation on October 14, 2026. The values called “baseline” below are starting assumptions or code references, **not measurements of Chuck**.

[Open trajectory map](http://127.0.0.1:5174/map.html) · [Open measurements and log import](http://127.0.0.1:5174/advanced.html)

## 1. What you are trying to identify

The model has three jobs: predict what leaves the robot, predict its flight, and account for the robot moving during release. Measure those in that order. A missed shot alone generally cannot tell you which part is wrong.

Start with geometry, actual launch elevation and ball exit speed. Add aerodynamic effects after those are repeatable. Add moving-shot corrections after stationary shots work. Keep a separate record of what was measured, what was assumed and what was only adjusted to improve outcomes.

The laptop searches sampled launch angles and speeds, retains scoring bands, picks a robust shot and fits quadratic patches in distance and radial velocity. The robot evaluates those fits, subtracts tangential velocity from the 3D launch vector, transforms for chassis tilt and converts the result to hood command and wheel RPS. It does not run a trajectory search while targeting.

The shipped baseline has unresolved fit regions and compensated mechanism-limit failures. It is not a calibrated or fully covered shooting solution. **Do not alter measured physics merely to make a coverage map greener.** First determine whether the failure is physical feasibility, optimizer resolution, fitting error or a real measurement mismatch.

## 2. The shortest useful order of work

1. Check the robot pose, hub location, turret offsets, azimuth zero and gyro signs.
2. Measure release height and fix a consistent wheel-radius definition.
3. Measure ball launch angle at hood commands **13° and 29.85°**; check the midpoint **21.425°**.
4. Measure exit speed against measured wheel RPS. Determine effective speed transfer.
5. Repeat identical releases to measure angle and speed variation.
6. Generate a baseline map using those values, with unmeasured aerodynamic corrections disabled.
7. Test stationary near, middle and far shots with both turrets. Record actual trajectories and timing.
8. Measure spin; fit drag, lift and spin decay only if the data supports them.
9. Determine per-turret differences, then release delay and moving-shot behavior.
10. Regenerate, validate, keep held-out trials and repeat the tests with representative balls and operating conditions.

Use the normal team startup and cleared shooting-area procedure. Manual measurement uses the existing `Tune/Use Tune Values` switch; no new startup mode or robot timing mechanism is required.

## 3. Equipment and a trustworthy measurement setup

Use a camera with a known capture frame rate, a stable camera support, a length reference in the ball’s flight plane, a way to measure height/angle, and a scale for ball mass. A second synchronized view is valuable for spin, yaw and 3D moving shots. Robot logs supply pose, motion and measured flywheel RPS; they do not measure ball exit speed, launch elevation or spin.

For stationary release measurements, point the camera perpendicular to the initial flight plane, level its image axes, and place the scale where the ball travels. Avoid a scale on the wall several feet behind the ball: perspective changes the conversion from pixels to meters. Keep the lens setting and camera location fixed within a session. Use sufficiently short exposure that the ball center and spin marker can be located reliably.

Verify the **capture** frame rate from the recording, not its slow-motion playback rate. If frames have variable timing, use each frame’s timestamp. Count dropped or duplicated frames. A known-rate recording can be checked by timing a visible external reference. Use a wide enough view to see release and enough subsequent motion to distinguish launch conditions from flight effects.

Record uncertainty. For example, if a 1 m reference spans 500 pixels, one pixel represents about 2 mm in that plane. That does not include parallax, lens distortion, center-picking error or timestamp uncertainty. Re-measure a few trials independently to estimate those errors. Very short frame spacing magnifies pixel error when calculating speed; very long spacing contaminates initial speed with aerodynamic changes.

Assign each ball an ID. Record turret, hood command, commanded RPS, **measured RPS at release**, battery/operating condition, ball ID, video filename and release frame. Leave unmeasured fields blank. Zero means a measured zero or an explicitly disabled correction; it must not mean “unknown.”

## 4. Units, axes and signs

Use meters, seconds, m/s, wheel revolutions per second and ball revolutions per second. Multiply RPM by `1/60` to obtain RPS. The simple map’s launch elevations and hood controls are degrees. The advanced trial fields `yaw` and `azimuth` are **radians**. Raw gyro and chassis angular speeds are radians and radians per second.

Robot coordinates follow the intended WPILib convention: +X forward, +Y left and +Z up. Positive yaw is counterclockwise viewed from above. The current code’s tilt transform is `world = Rz(heading) × Ry(pitch) × Rx(roll) × robot-vector`. Under that mathematical convention, positive roll raises the left side; positive pitch rotates the robot’s forward axis downward. **Check the sensor’s actual signs on the robot** rather than assuming its displayed “pitch” uses the same sign.

Radial velocity is positive **toward the hub**. Tangential velocity is positive to the left of the turret-to-hub direction. The robot includes translation and the turret’s rotation-induced velocity before calculating these components. At a fixed physical release point:

```text
velocity_at_turret = chassis_field_velocity + omega × field_rotated_turret_offset
radial = velocity_at_turret · unit_vector_toward_hub
tangential = velocity_at_turret · unit_vector_left_of_hub_direction
```

The coordinate convention is documented by [WPILib](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html). The exact transform and velocity equations above describe this checkout; verify them with controlled physical motion before fitting moving-shot parameters.

## 5. Every adjustable factor in this version

All robot dashboard paths below begin with `ShotModel/`. “Regenerate” means create a new version-2 map on the laptop, put the model in the robot project’s deploy directory, and load it through the normal deployment/restart process. The app never deploys it.

| Factor | Baseline | Robot dashboard suffix | Map input / action |
|---|---:|---|---|
| Release height | 0.445 m | `Setup/Release Height M` | `height`; regenerate |
| Wheel radius | 0.0508 m | `Setup/Wheel Radius M` | `radius`; regenerate |
| Elevation at hood 13° | 77° | `Tune/Elevation At Hood Min Deg` | `angleMin`; regenerate |
| Elevation at hood 29.85° | 60.15° | `Tune/Elevation At Hood Max Deg` | `angleMax`; regenerate |
| Effective speed transfer | 1 | `Tune/Ball Speed Transfer` | `transfer`; regenerate |
| Drag coefficient | 0, disabled | `Advanced/Drag Cd` | `drag`; regenerate |
| Ball spin / wheel revolution | 0, disabled | `Advanced/Ball Spin Per Wheel Rev` | `spinRatio`; regenerate |
| Effective Magnus coefficient | 0, disabled | `Advanced/Magnus Coefficient` | `lift`; regenerate |
| Spin decay | 0 s⁻¹ | `Advanced/Spin Decay Per Sec` | `spinDecay`; regenerate |
| Release-state delay | 0 s | `Advanced/Release Delay Sec` | Robot only; live, then retest |
| Left/right wheel multiplier | 1 each | `Left/Speed Multiplier`, `Right/Speed Multiplier` | Robot only; live, then retest |
| Left/right hood trim | 0° each | `Left/Hood Trim Deg`, `Right/Hood Trim Deg` | Robot only; live, then retest |
| Left/right aim trim | 0° each | `Left/Aim Trim Deg`, `Right/Aim Trim Deg` | Robot only; live, then retest |
| Release-angle standard deviation | 0.5°, assumed | No matching dashboard input | `sigmaAngle`; regenerate |
| Exit-speed standard deviation | 0.15 m/s, assumed | No matching dashboard input | `sigmaSpeed`; regenerate |

The two names `angleMin` and `angleMax` refer to the **hood command endpoints**, not sorted physical elevation. With the baseline, `angleMin = 77` is numerically larger than `angleMax = 60.15`.

The nine physical map/dashboard values must agree; mismatches explicitly invalidate hub targeting. The variability values affect robust shot selection and therefore also require new fits, although the Java settings-matching check does not compare those two inputs. Per-side trims and delay are applied at runtime and are not included in the packaged flat, neutral-trim coverage report.

## 6. Release height — measure the ball center at separation

**What it means:** vertical distance from the floor reference to the ball center when the ball leaves its last controlling contact. It is not the top of the wheel, turret pivot height or ball center while sitting in the mechanism.

1. Put the robot on a representative, level surface at its normal ride height and loading.
2. Mark a vertical length reference in the release plane. Film a clean stationary release from the side.
3. Identify the last frame of controlled contact and first clearly free-flight frame. Estimate the ball-center position at separation by interpolating between them.
4. Measure center height above the same floor reference used for the hub. Repeat at both hood endpoints and the midpoint, on both turrets.
5. Enter a representative measured height if its changes are small compared with the scoring margins. Record the spread as an unmodeled geometry error.

**If height moves substantially with hood angle:** the current map uses one height. Do not average away a large systematic change or hide it in drag. A hood-dependent release-position model needs an implementation change and new fits.

The current tilt compensation rotates the launch direction, but does not fully model suspension motion or a hood/tilt-dependent 3D release-origin displacement. Large pitch/roll or chassis deflection needs additional measurement and possibly a geometry-model change.

## 7. Wheel radius, compression and slip

Choose a consistent radius convention and keep it fixed while identifying transfer. A useful simple convention is the measured nominal wheel contact radius; let effective transfer absorb compression and slip. If you instead use a measured loaded rolling radius, record exactly how it was obtained and do not switch definitions between sessions.

Measure diameter across the wheel’s contact surface in several orientations and divide by two. Inspect wear, asymmetric compression, loose tires and inconsistent ball contact. If there are different wheel surfaces or multiple contacts, identify which measured shaft RPS corresponds to the radius used in the equation.

```text
wheel_surface_speed = 2 × pi × radius_m × measured_wheel_RPS
```

Confirm that logged RPS is the relevant wheel-shaft speed after any gearing. A configured gear ratio is not a measurement that the installed mechanism matches it. Check with an independent rotation reference where possible.

**Identifiability limit:** ordinary exit-speed measurements identify the product `radius × transfer`. You cannot independently solve radius, compression loss and slip from those same observations. Fix radius from geometry first. Separate compression/slip studies require additional measurements of deformation, contact motion or wheel/ball surface velocity; they are not separate knobs in this version.

## 8. Launch-angle mapping — the two endpoint measurements

Use `Tune/Use Tune Values = true` and the existing manual hood/flywheel controls. Follow the normal turret startup procedure first. The hood value in telemetry is a command, not measured physical hood position or ball elevation.

1. Command hood **13°**. At a steady, repeatable wheel speed, film several clean releases.
2. Locate ball centers over the first few free-flight frames. Convert coordinates into meters in the flight plane with up positive.
3. Estimate initial horizontal and vertical velocity at the separation instant. For a short initial interval with negligible aerodynamic change, fit:

```text
x(t) = x0 + vx0 × t
z(t) = z0 + vz0 × t − 0.5 × 9.81 × t²
launch_elevation_deg = atan2(vz0, vx0) × 180 / pi
```

4. Repeat at hood **29.85°**. Enter the two mean physical elevations in the matching dashboard and map fields.
5. Check hood **21.425°**, and preferably additional intermediate commands. Compare measured elevation with the straight line between the endpoints.
6. Repeat a representative setting at another wheel speed, and on the other turret, to check whether ball release direction changes with speed or contact conditions.

Use the shortest interval that gives a stable estimate. With measurable drag/lift even near release, fit the full trajectory or compare several short fitting windows and extrapolate to release; do not treat a later average velocity as initial velocity.

The current conversion is linear:

```text
elevation(h) = angleMin + (h − 13) × (angleMax − angleMin) / (29.85 − 13)
hood(elevation) = 13 + (elevation − angleMin) × (29.85 − 13) / (angleMax − angleMin)
```

**Ready to use when:** repeats are stable, the endpoints are independently measured, and midpoint/intermediate residuals are small relative to the physical scoring-angle margins. A consistent nonlinear residual requires a richer mapping in code. A constant side-to-side difference may be handled by a hood trim; a change in slope cannot.

## 9. Effective wheel-to-ball speed transfer

Keep the robot stationary, radius fixed and hood mapping measured. Film releases at two or more steady wheel speeds and several hood commands. Record **measured** wheel RPS at each release, including wheel droop; do not substitute the setpoint.

For each shot:

```text
ball_exit_speed = sqrt(vx0² + vy0² + vz0²)
transfer = ball_exit_speed / (2 × pi × radius × measured_wheel_RPS)
```

A single side camera only gives full speed when the shot stays in its calibrated plane. If there is yaw or out-of-plane motion, use a second view or calibrated 3D reconstruction.

**Illustrative calculation, not a Chuck measurement:** radius 0.0508 m, measured speed 50 RPS and measured exit speed 11.2 m/s give surface speed about 15.96 m/s and transfer about **0.702**. Enter your measured ratio, not this example.

Plot measured exit speed against `2πr × measured RPS`. Fit a through-origin slope if the residuals support proportional transfer. Check the intercept instead of forcing it away blindly: a repeatable nonzero intercept, changing slope with speed, or strong hood dependence means the single-ratio model is inadequate over that range.

Inspect residuals by turret, hood, wheel speed, ball ID and battery/feeding condition. Use clean steady-state trials first. A measured wheel-speed tracking problem is a mechanism/control problem; transfer should not conceal it.

**Outcome-only fallback:** after fixing geometry and angle mapping, decreasing transfer commands more wheel RPS for the same desired ball speed; increasing transfer commands less. Use small changes and repeated shots. Label this “outcome tuned,” not “measured transfer.” Regenerate the map after changing it. Long/short misses alone cannot separate transfer from launch angle, drag or pose errors.

## 10. Release variability — angle and speed standard deviations

These tell the optimizer which scoring margins are valuable. They are not arbitrary aggressiveness sliders.

1. Hold turret, hood, requested wheel speed and robot state constant. Gather at least ten clean measured releases as a practical starting sample, then increase the sample if estimates are unstable.
2. Measure each launch angle and exit speed using the same method. Use actual measured wheel RPS to identify tracking-related variation.
3. Calculate sample standard deviations:

```text
mean = sum(values) / n
sample_stddev = sqrt(sum((value − mean)²) / (n − 1))
sigmaAngle = sample_stddev(elevation_degrees)
sigmaSpeed = sample_stddev(exit_speed_m_per_s)
```

4. Repeat near several expected operating conditions. Do not calculate one standard deviation from shots intentionally taken at different hood/RPS settings; that mostly measures changes in the mean.
5. Record within-condition variation, between-ball variation and measurement uncertainty separately. If combining groups, remove their different predicted means before pooling residuals. Until a condition-dependent uncertainty model exists, select a conservative representative value rather than claiming a low-noise value covers every condition.

For example, ±1° of angle margin and ±0.3 m/s of speed margin have different usefulness if release scatter is 0.2° versus 0.8°. The optimizer uses the smallest of the four normalized margins. The current model treats angle and speed variation as separate scalars; it does not model correlation, heavy tails or a guaranteed success probability.

Camera measurement noise is included in raw observed scatter. Estimate it with repeated independent annotations. Do not subtract variances unless the independence assumptions and sample quality support that calculation. Keep realistic positive values; zero is not a valid variability input.

## 11. Ball mass, size, shape and condition

**Mass:** weigh several identified balls with a checked scale, with no support carrying part of their weight. Record mean, range and scale resolution. The current reference is **0.20335424 kg**.

**Radius:** measure diameters along several axes while compressing the ball as little as practical. The current reference is **0.075 m**. Aerodynamic area is `π × radius²`; collision clearance also uses ball radius. A deformable or non-spherical ball cannot be fully described by one radius.

**Condition:** note new/worn, dry/damp, temperature exposure, visible damage, softness and marker/tape changes. Keep any spin marker small enough that it does not materially change mass, surface or balance. Compare marked and unmarked outcomes if the marking is substantial.

Run repeatability tests with multiple representative balls. If different populations need meaningfully different transfer or flight parameters, maintain separately labeled configurations and validate each. The simple runtime has no ball-condition sensor or automatic per-ball selection.

**Where to change these:** mass and ball radius are currently fixed reference values in the version-2 generator/physics code, not map-page controls or robot dashboard knobs. The vacuum fast path also contains explicit geometry constants. Changing reference metadata in exported JSON does not regenerate coefficients or change that physics. Updating these factors requires consistent code changes, tests and regeneration.

## 12. Hub location, height, opening and rim geometry

Verify the target center against the same field coordinate origin used by localization. The current blue-origin hub center is **(4.6256194, 4.0346376) m** and the modeled opening plane is **1.83 m** above the floor reference. These are source references to verify against the actual field/model, not new survey results.

The simple app’s blue-origin alliance-zone rectangle is X = 0–5.1816 m and Y = 0–8.0756125 m. These are coverage-sampling bounds, not a substitute for checking the actual navigable field and robot clearance. For opposite-alliance tests, verify the existing alliance transform against surveyed field landmarks rather than adding an extra manual flip.

Survey a known robot position and heading, then independently measure turret-to-hub geometry. Repeat from multiple sides. If all shots appear consistently offset in field space, fix target/localization geometry before using turret trims.

Measure the actual usable opening, rim cross-section and target shape. The simulator currently uses a circular opening/ring model with hub ring radius **0.56 m** and rim-tube radius **0.025 m**. For a torus, ring radius is the distance from opening center to the tube centerline; it is not automatically the clear inner radius.

The modeled centerline clearance at the opening plane includes both ball and tube radius:

```text
distance_from_hub_center + ball_radius + rim_tube_radius < hub_ring_radius
```

The simulation also checks the approach/departure path for a rim touch. A ball center crossing inside the opening is insufficient if the ball clips the near or far rim. Rim touches are excluded from the robust region even if a real bounce sometimes scores.

If the actual opening is noncircular or has other obstacles, the circular approximation may be wrong in an azimuth-dependent way. Do not fit a false drag coefficient to compensate. Hub dimensions are fixed in the current simple generator, including its fast vacuum path; changes require a consistent physics implementation update and regenerated fits.

## 13. Turret offsets and the physical release point

Current robot-frame turret offsets are:

| Turret | X, forward m | Y, left m |
|---|---:|---:|
| Left | −0.18415 | +0.19685 |
| Right | −0.18415 | −0.19685 |

Measure from the robot pose origin used by odometry, not a bumper corner. Confirm the physical turret identities and that positive Y really maps to the left unit. Check the origin against the drivetrain geometry.

Measure both the turret rotation axis and the actual ball release point. This version uses a fixed 2D offset to represent the turret’s release origin. If release is materially displaced from the rotation axis and that displacement rotates with turret aim, the fixed-offset approximation needs a code change. Record that error before tuning moving shots.

Validate offsets by rotating the chassis at a known rate while tracking the release point. The additional velocity should follow `omega × offset`; opposite Y offsets produce different X velocity contributions. A useful numerical check: at omega = +1 rad/s and heading zero, the left offset contributes roughly (−0.19685, −0.18415) m/s, while the right contributes (+0.19685, −0.18415) m/s. These are predictions from the source offsets, not measured velocities.

Offsets are source constants, not dashboard trim controls. Fix incorrect geometry in the canonical robot/generator definitions and revalidate both turrets. A yaw trim cannot repair the wrong rotational velocity.

## 14. Pose, field velocity and angular velocity

Stationary trials: place the robot at several independently measured field locations and headings. Compare logged pose and heading against those references. Check field origin and alliance transformation before collecting a large dataset.

Translation trials: drive a measured straight path at steady speed; compare external displacement/time with the logged **field-relative** velocity. Repeat in the opposite direction and with the chassis facing a different heading. Robot-relative velocity accidentally treated as field-relative often produces heading-dependent misses.

Rotation trials: rotate through a measured angle while recording time and gyro/pose heading. Compare measured angular rate with logged rad/s. Repeat both directions and check sign. Avoid estimating a derivative across an angle wrap without unwrapping it.

The chassis shooting translation limit in this checkout is **2.5 m/s**. The coverage reference uses angular speed up to about **6.25434 rad/s**. The map accepts turret release-point speed up to **4.25 m/s** after adding rotation-induced motion; this is not permission to command the chassis above its shooting speed limit.

Verify actual speeds rather than assuming the commanded chassis setpoint was achieved. Acceleration, localization latency, drivetrain slip and stale velocity estimates are not independently adjustable factors in the current shot model. Diagnose them from external motion and logs before fitting release delay.

## 15. Pitch and roll

On level ground, compare the sensor’s pitch/roll with an independent level reference. Raise the front and then the left side by known, modest amounts; measure the resulting inclination and verify axes, scale, sign and mounting offsets.

The current code expects the mathematical transform in Section 4. Do not use an opposite sign simply because a vendor display labels it “pitch.” A wrong sign can make compensation double the error. Correct the sensor transform/mounting or code, then rerun the vector tests.

After level stationary shots are reliable, repeat the same target geometry at controlled positive/negative pitch and roll. Record actual ball release direction, wheel tracking and commanded hood. Check that the compensated request stays inside hood and flywheel limits.

The code transforms desired launch direction into the tilted robot frame. It does not automatically identify flex, suspension-height changes or a moving 3D release origin. The packaged alliance coverage is for a flat chassis; passing software vector tests is not a tilted-shot hardware validation. The reference 5° tilt value in the older workspace is a test setting, not proof of a validated tilt envelope.

## 16. Backspin transfer and spin direction

Measure spin directly before trying to identify Magnus lift.

1. Add a small visible, asymmetric marker to a representative ball. Use a view that reveals rotation and avoids confusing two identical appearances per revolution.
2. Film several free-flight frames near release. Track marker orientation and count full revolutions plus the partial revolution between endpoints.
3. Calculate `ball_rev_per_s = revolutions / elapsed_seconds`. Use more than two frames where possible to detect aliasing and identify direction.
4. Divide initial ball spin by **measured wheel RPS at release**:

```text
spinRatio = signed_initial_ball_rev_per_s / measured_wheel_RPS
```

5. Repeat across hood, wheel speed and both turrets. If this ratio changes systematically, one constant is inadequate.

Positive `spinRatio` means backspin in this code: for a shot initially along +X, the assumed spin axis is −Y, producing upward Magnus acceleration. Negative is topspin. Verify with the marker and observed motion; an unsigned spin measurement is insufficient.

Example only: 20 ball rev/s at 50 measured wheel RPS gives spinRatio = +0.4 for backspin. Do not enter that example without measuring it. Spin is converted to rad/s internally with `2π`.

Setting spinRatio alone does not add lift when `lift = 0`. Conversely, increasing `lift` with spinRatio still zero cannot create the modeled Magnus force. Flight curvature alone generally identifies a combination of spin and lift; direct spin measurement separates them.

## 17. Drag coefficient

Use measured launch conditions and full flight observations after fixing geometry, angle mapping, speed transfer and spin direction. Prefer multiple trajectories spanning the actual speed and range envelope. Hold out some trajectories for validation.

The model uses relative air velocity `u = ball_velocity − wind`, area `A = πR²`, and:

```text
a_drag = −0.5 × air_density × Cd × A / mass × |u| × u
```

This is the conventional quadratic drag form; see [NASA’s drag equation](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/drag-equation/). A generic sphere coefficient is not a measured foam-ball coefficient. NASA also notes that [sphere drag depends on flow conditions](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/shape-effects-on-drag/).

**Preferred identification:** track ball positions and timestamps throughout several flights. With each measured initial state fixed, try candidate Cd values, integrate the same physics used by the generator, and minimize position/timing residuals. Compare full trajectories, not only final range. Then test held-out flights. Do not refit initial speed freely for every shot until it absorbs any drag error.

A direct, noisier diagnostic is possible if 3D velocity and acceleration are reliable:

```text
a_non_gravity = measured_acceleration + [0, 0, g]
Cd ≈ −2 × mass × (a_non_gravity · u) / (air_density × A × |u|³)
```

That projection assumes the other aerodynamic force is perpendicular to `u` and there are no unmodeled forces. Differentiating video positions twice is noisy; forward fitting is usually more useful.

Keep density, radius and mass fixed at measured/reference values while fitting Cd. Otherwise the same trajectory can be explained by different combinations. A residual that grows with distance may suggest drag, but is not proof: launch-angle bias, initial-speed bias and timing errors also grow into larger range errors.

If only makes/misses are available, a small positive Cd can be outcome tuned with repeated short/long trials, but label it an effective fitted correction with weak identification. Do not claim you independently measured aerodynamics from one distance.

## 18. Magnus lift coefficient — this code’s convention matters

The current implementation uses:

```text
omega(t) = signed_spin_rev_per_s × 2π × exp(−spinDecay × t) × spin_axis
b = 0.5 × air_density × A × R / mass × |u| × (omega × u)
a_Magnus = lift × b
```

**`lift` is an effective coefficient for that exact equation. It is not the usual dimensionless aerodynamic lift coefficient CL.** Because this implementation includes an extra `|u|` factor alongside `omega × u`, the scalar has inverse-velocity units (s/m) when the other quantities use SI. Do not paste a textbook CL or a coefficient from another simulator into this field.

To determine it, measure spin independently, fix the launch state, and estimate drag first. Compare measured 3D flight shape with simulations while varying only `lift`; use multiple speeds and measured spins. Backspin should produce the expected lift direction. If the sign is wrong, fix spin convention rather than using a negative coefficient to conceal it. The current simple controls accept nonnegative lift; spin sign carries backspin/topspin direction.

For sufficiently good acceleration data, a code-consistent least-squares estimate is:

```text
residual_accel = measured_acceleration + [0, 0, g] − predicted_drag_accel
lift ≈ sum(b · residual_accel) / sum(b · b)
```

This diagnostic requires synchronized spin, velocity and acceleration and a trustworthy spin axis. When `b` is small or nearly unchanged across all trials, the parameter is poorly identified. Forward trajectory fitting with measured spin is less sensitive to differentiation noise.

The model assumes a horizontal spin axis perpendicular to launch azimuth. Significant sidespin, tilted spin axes or changing contact conditions need more measurement and potentially a richer model. Nonzero lift also makes tangential and tilted-shot validation especially important; vector compensation is not a guarantee that aerodynamic forces stay identical.

## 19. Spin decay

Spin decay models exponential loss of angular speed during free flight:

```text
spin(t) = spin0 × exp(−lambda × t)
lambda = ln(|spin1| / |spin2|) / (t2 − t1)
```

Measure marker rotation in several short windows along the same flight. Use actual timestamps and avoid temporal aliasing. Estimate `ln(abs(spin))` against time; the negative fitted slope is lambda in s⁻¹. Compare repeated flights before accepting a trend.

Example only: a decline from 20 to 16 rev/s over 0.5 s gives lambda about 0.446 s⁻¹. Constant spin within measurement uncertainty supports leaving the correction at zero; it does not prove decay is physically absent.

If the spin changes sign, becomes poorly visible or does not follow an exponential trend, this simple formula is not an adequate description. Do not fit decay from an assumed spin measurement. If lift is disabled, spin decay has no aerodynamic effect on the computed trajectory, so scoring outcomes cannot identify it in this model.

## 20. Air density, wind and environmental conditions

The simple generator currently fixes density at **1.2041 kg/m³**, gravity at **9.81 m/s²**, and wind at **[0,0,0]**. These are not map-page knobs.

Record ambient temperature, pressure and noticeable airflow. A dry-air density estimate is `rho ≈ pressure_Pa / (287.05 × temperature_K)`; use station pressure rather than a sea-level-corrected weather report. Humidity changes the estimate; precision work needs a moist-air calculation and suitable measurements. If density remains fixed, fitted Cd/lift are effective values for that reference density.

Check airflow near the release, through the flight area and around the hub. Opposite-direction drift that changes with location may come from airflow or localization, not a fixed turret aim offset. The current radial-only, still-air model cannot represent arbitrary field-direction-dependent wind with one distance/radial fit. Do not assume changing a wind value in the old advanced workspace updates the version-2 robot map.

Do not tune gravity to make shots land correctly. If a controlled free-flight gravity check disagrees strongly with 9.81, check video timing, scale and camera projection first. Gravity is a known physical reference for this application, rather than a useful shooter correction knob.

## 21. Release delay and timestamp alignment

This parameter is the time between the robot state used for aiming and the effective ball separation event represented by the model. It is **not flight time** and is not automatically the time from an operator pressing “shoot” to the ball leaving.

First determine what the mechanism actually does: wheels may already be spinning, and the feeder’s request may precede contact by a variable amount. Identify an observable event that marks the relevant command/aim state. If there is no synchronized observation of that event, do not pretend the delay is directly measured.

A practical measurement procedure:

1. Record video of separation and robot telemetry in the same run.
2. Find a visible event also recorded in the log, or use another verified synchronization method. A synchronized pair of cameras can connect views; an unsynchronized wall clock is not enough.
3. Establish the mapping between video frame timestamps and robot-log time. Use two or more anchors if drift is possible. Record offset and uncertainty.
4. Mark separation and the relevant command/state event. Compute their time difference for repeated shots under representative feeding conditions.
5. Test the resulting value while translating both toward and away from the hub and while rotating both directions. Keep measured launch conditions fixed.

The robot predicts release position and heading as current state plus constant velocity/angular velocity times this delay. That approximation does not capture arbitrary acceleration during the interval. If latency varies widely or motion estimates are stale, one constant is insufficient.

WPILOG record timestamps use microseconds, while this app’s marked shot event is entered in **robot seconds**; see [WPILib data logging](https://docs.wpilib.org/en/latest/docs/software/telemetry/datalog.html). Convert by dividing microseconds by 1,000,000. Playback position in a video file is not automatically robot time.

The importer takes the last recorded channel value at or before the event. Signals may have different update times. Inspect channel freshness around release, especially during wheel droop or rapid acceleration. A correct timestamp unit does not guarantee synchronized or equally fresh channels.

If timing cannot be synchronized, use small, reversible delay changes and paired steady-motion tests. Label this outcome tuning. Increasing delay moves the predicted release state farther along the robot’s current motion; there is no universal “short means increase delay” rule.

## 22. Per-turret speed, hood and aim corrections

Start all six trims neutral. Determine shared launch physics before using them. Fix swapped identities, bad encoders, wheel tracking, wrong geometry and mechanical looseness first.

### Speed multiplier

The runtime command is:

```text
wheel_RPS_command = desired_ball_speed / (2π × radius × global_transfer) × side_multiplier
```

At the same commanded hood and measured wheel RPS, compare both turrets’ exit speeds. If the side-specific transfer is `k_side` and the common chosen transfer is `k_global`, a proportional model gives:

```text
side_multiplier = k_global / k_side
```

Alternatively, at a fixed requested physical shot and approximately linear transfer, `new_multiplier = old_multiplier × desired_exit_speed / observed_exit_speed`. Confirm measured wheel tracking and repeat before accepting it. This increases command RPS when that side needs more command to obtain the requested ball speed.

One scalar cannot correct a side-specific nonlinear speed curve or hood-dependent compression. A changed multiplier also changes the real shot/spin behavior and needs revalidation; it is not part of the exported neutral-trim simulation.

### Hood trim

Measure launch elevation on each side at matched commands. Let the shared mapping slope be:

```text
slope = (angleMax − angleMin) / (29.85 − 13)
new_hood_trim = old_hood_trim + (desired_elevation − observed_elevation) / slope
```

With the baseline negative slope, a **positive hood trim lowers physical elevation**. Verify your measured slope before applying that sign. Recheck more than one hood command. A constant trim can correct an offset, not a different slope or nonlinear mapping.

### Aim trim

With verified pose, hub center and turret zero, measure the initial ball’s horizontal direction during stationary level shots. Calculate the signed angular correction from measured direction to intended direction and add it to the side’s aim trim. Positive trim commands a counterclockwise/leftward change in robot coordinates.

For small stationary yaw errors, a lateral miss `e_left` at distance `d` suggests approximately `delta_yaw = −atan2(e_left, d)`; convert to degrees for the dashboard. This is an approximate diagnostic, not a replacement for measuring release direction. Test several distances and chassis headings. A field-fixed error or range-dependent error is not necessarily a turret-zero error.

Aiming at a marked target with a mechanical sight is useful for checking azimuth zero, but the launched ball may depart at a different horizontal angle. Keep the mechanical zero check and the observed ball-direction correction separate.

## 23. Logs and a measurement worksheet that will remain useful

The companion release CSV uses the existing advanced workspace’s measurement fields. Fill required hood and measured wheel RPS before importing a row. Blank ball-speed/elevation/spin fields stay unknown. The run ledger contains additional experimental context and is a separate record, not a direct-import file.

For each release, keep:

| Item | What to record |
|---|---|
| Identity | Trial ID, date/session, ball ID, turret, operator/annotator |
| Configuration | Model/config revision, radius convention, global parameters and side trims |
| Timing | Capture FPS or frame timestamps, release frame, robot seconds, sync method/error |
| Mechanisms | Hood command, commanded wheel RPS, measured leader/follower RPS at release |
| Ball measurements | Initial elevation, full exit-speed magnitude, spin direction/rate, uncertainties |
| Robot state | Pose, heading, field velocity, angular velocity, roll/pitch and channel freshness |
| Flight | Position/time observations, descending opening-plane TOF, center crossing location |
| Outcome | Clean score, rim touch, short/long/left/right miss, test interruption |
| Conditions | Surface, loading, ball condition, temperature/airflow and observed anomalies |

In the advanced workspace, open Logs, import `.wpilog`, enter the aligned release timestamp in seconds, select the turret, and attach telemetry to a trial. Required measured-wheel and hood-command channels must exist. A ferry event is rejected for hub calibration. Missing channels are not measured zeros.

Then add externally measured ball elevation, speed, spin and flight observations in Measurements. Preserve the original video/log and raw annotations; fitted parameters should not be your only surviving evidence. Keep clean calibration trials separate from moving trials until relative release velocity is reconstructed.

**Important version distinction:** the advanced workspace can fit richer transfer/aerodynamic relationships and export an older version-1 layered model. That model is incompatible with the current version-2 robot loader. Use it for analysis, measurement management and diagnostics; transfer compatible measured constants manually into the version-2 map page. Generating or editing an old project does not automatically update the new map.

### Other controls visible in the older advanced workspace

The older workspace exposes additional physics fields. Most refer to factors already explained above, but these scaling controls need a separate explanation:

| Older control | Meaning in that workspace | Relation to the current robot map |
|---|---|---|
| `speedScale` | Multiplies the older calibrated speed relationship | No separate version-2 knob. Use measured transfer and supported side multipliers; do not apply the same correction twice |
| `angleOffset` | Adds an offset to the older launch-angle calibration | No automatic transfer. Determine actual endpoint elevations for the version-2 map |
| `spinScale` | Scales the older inferred spin relationship | No automatic transfer. Determine the signed version-2 spin ratio from measurements |
| `latency` | Release-state prediction setting in the older model | The version-2 robot uses its own `Advanced/Release Delay Sec`; editing the old project does not change it |
| Wind / mass / size / density / hub geometry | Editable physics references in the older project | The simple version-2 generator uses its own fixed references; older project edits do not propagate |

The older default Cd is 0.47 and its latency reference is 0.05 s. The simple version-2 baseline instead disables drag and starts robot release delay at zero. Neither pair is a measurement. Keep the project/version visible in every screenshot, export and experiment record so those defaults are not mistaken for applied robot settings.

## 24. Moving-shot tests and the radial/tangential split

Complete stationary tests first. Use repeatable, nearly constant motion through a measured test location. Begin slowly, then repeat at 1.25 and 2.5 m/s where the physical setup and model outputs permit.

Test toward, away, left-across, right-across and diagonal motion; then combine translation with both signs of chassis rotation. Repeat on both turrets and at multiple field positions/headings. Use actual measured motion at release, not the driver’s intended direction.

For externally measured moving release speed, subtract the turret release-point velocity from the ball’s world velocity:

```text
ball_relative_exit_velocity = measured_world_ball_velocity − release_point_velocity
```

Otherwise you may accidentally count chassis motion as improved wheel transfer. Measure the ball’s relative elevation from this relative vector as well.

The map already accounts for radial velocity. Runtime tangential correction starts from the fitted relative radial/vertical components:

```text
x = fitted_speed × cos(fitted_elevation)
y = −tangential_velocity
z = fitted_speed × sin(fitted_elevation)
new_speed = sqrt(x² + y² + z²)
new_elevation = atan2(z, sqrt(x² + y²))
yaw_offset = atan2(y, x)
```

Do not subtract radial velocity a second time. Sideways motion changes speed and elevation as well as turret yaw. Chassis tilt is applied afterward when converting the desired world vector into robot-frame commands.

If one turret fails mainly during rotation, inspect its offset/identity and azimuth tracking. If both fail in the same field direction, inspect field velocity, pose or airflow. If errors reverse with acceleration but not with steady speed, inspect motion estimation and variable release latency before changing drag.

## 25. Numerical and fitting factors — these are software settings

The version-2 sweep currently samples launch elevation at spacing no greater than **0.5°** and exit speed at spacing no greater than **0.15 m/s**, refines speed boundaries and refines the selected robust shot. “All possible shots” means all pairs at that numerical sampling resolution, not a mathematical proof over a continuous domain. A very thin scoring band can be missed by a coarse grid.

The current domain is **0.65–7.5 m** and radial velocity **−4.25 to +4.25 m/s**. Verify that measured/predicted release positions for your actual delay, offsets and motion remain inside it. The domain is a tested input region, not a strategy recommendation to disable useful distances. Extend and validate it in software when necessary; do not extrapolate untested coefficients silently.

Fits use normalized terms `1, d, r, d², d*r, r²`. Seven outputs are launch elevation, exit speed, flight time, lower/upper elevation margins and lower/upper speed margins. Smaller quadratic patches are used where a larger fit fails. The current generator has a finite subdivision limit; it can leave a gap even when the underlying optimizer finds a scoring shot.

Classify failures before changing anything:

| Observation | Likely work needed |
|---|---|
| No sampled scoring band | Check physics/mechanism feasibility, inputs, search resolution and collision geometry |
| Sweep scores, but fitted map has a hole | Improve optimizer consistency, fit subdivision/resolution or validation; do not falsify measurements |
| Fit scores in simulation, real release state differs | Fix launch transfer/mapping or mechanism tracking |
| Measured release matches, real flight differs | Examine aerodynamic/environmental/collision assumptions |
| Compensated hood/RPS rejected | Check physical limits and mapping; a different robust shot or mechanism range may be required |

The generated map accepts patches only when sampled trajectory re-simulation passes and fitted errors are below 10% of the corresponding **reference simulated** margins; the time-fit budget is 0.01 s. Measured scoring margins must be established independently before calling this a measured-error guarantee.

The RK4 integrator uses a nominal **0.01 s** step with a shorter collision-related travel cap. Vacuum search uses an analytic trajectory and numerical rim-clearance checks. Numerical settings are validated by reducing step/grid spacing and comparing scores, boundaries and selected shots. If the result changes materially, improve numerical resolution; do not tune real-world coefficients to compensate for a numerical error.

The older display simulator includes a restitution value **0.2** for visualized rim bounces. Clean-score selection rejects rim contact, so changing restitution cannot improve the valid scoring region. Restitution would require controlled impact tests if bounce prediction became a goal; it is not a current shooting-calibration priority.

## 26. Regenerate and load the correct model

1. Export your input/config record and preserve the measurement session that supports it.
2. On the version-2 map page, enter the measured geometry/transfer values and justified aerodynamic coefficients. Enter measured variability. Unknown aerodynamic corrections may stay disabled, explicitly labeled as such.
3. Sweep representative scenarios. Inspect the red/green boundaries and selected shot; check the tangential re-simulation result.
4. Generate the distance × radial map. It runs locally in a worker and can be canceled. Review fit gaps and the alliance motion coverage report.
5. Download `chuck_shot_model.json`. Do not use the advanced workspace’s version-1 export.
6. Put the file at `Rebuilt26/src/main/deploy/chuck_shot_model.json`. Have the normal team process handle deployment and restart when appropriate. This guide/app does not perform deployment.
7. Set the nine dashboard physics values to the same numbers used for generation. Check `ShotModel/Map Status`, `ShotModel/Settings Valid`, and the left/right `Model Valid` and `Model Status` fields.
8. Use `ShotModel/Save Settings` to persist dashboard values in robot Preferences. Saving Preferences does not save, regenerate or reload the model file.
9. Retest after loading. The model is read once during startup; replacing the source file or downloading a file in the browser does not update a running robot.

Per-side trims and release delay remain live. Save them explicitly and record them with every validation run. `Reset To Baseline` resets live dashboard settings; it does not regenerate the map or erase saved Preferences unless you then save. It may make an otherwise calibrated file mismatch the dashboard.

The current exporter leaves `calibrated: false` and `physicalValidation: false`. Entering measured numbers or obtaining a green simulation does not automatically change those flags or establish hardware validation. Keep an independent signed-off trial record; do not flip flags merely to claim success.

## 27. A practical first data-collection plan

Treat these counts as a starting experiment, not an accuracy guarantee.

**Session A — geometry and release:** survey the target/robot origins and check sensor signs. On each turret, test both hood endpoints and the midpoint at two steady wheel speeds, five repeats per combination: 30 measured releases per turret. Measure exit angle, speed and initial spin when possible. Add speed/hood points if residuals reveal nonlinearity.

**Session B — repeatability:** choose representative near/middle/far operating conditions and expand repeated groups to at least ten measured releases each. Estimate angle/speed scatter without mixing different commanded means. Include several identified balls.

**Session C — flight:** with launch transfer fixed, record full trajectories at varied physical speeds, elevations and distances. Keep measured initial conditions for each shot. Fit drag and then supported spin/lift/decay parameters; reserve separate shots for validation.

**Session D — motion:** test paired toward/away/across motions at increasing speeds, both rotation directions and both turrets. Align video and log release events. Determine whether a repeatable delay explains the residuals. Then test controlled pitch/roll.

**Session E — coverage:** repeat stationary and moving shots across the actual navigable alliance-zone grid, including near boundaries and intermediate positions. Keep failed positions visible. Do not test only cells that already look green in simulation.

A single make is not a scoring boundary. For physical margins, hold other variables constant and bracket the onset of reliable short/long/rim misses as you vary physical angle or exit speed. Repeat near each proposed boundary, use actual measured release values, and define the team’s acceptable make-rate/uncertainty criterion before declaring a margin. With noisy shots, the boundary is probabilistic. Report makes/attempts and uncertainty rather than treating 20/20 as proof of 100% reliability.

## 28. How to know a factor is actually determined

A useful factor record includes its value, units, method, uncertainty, supported operating range, number of trials, raw evidence and independent validation results. If changing another parameter can explain the same observations equally well, record that ambiguity.

| Factors that can be confused | How to separate them |
|---|---|
| Radius / transfer / compression / slip | Measure and fix radius; use exit-speed data for combined transfer; extra contact measurements are needed for the separate losses |
| Launch angle / speed / drag | Measure initial direction and speed externally; use later trajectory shape/timing for drag |
| Spin / lift | Measure signed spin; then fit the code-specific lift coefficient |
| Spin decay / lift strength | Measure spin at several times; use trajectories at varied times/speeds |
| Density / Cd / mass / area | Measure or fix density, mass and size; identify Cd relative to those choices |
| Release delay / pose latency / velocity error | Synchronize events; measure motion independently; compare steady motion with acceleration |
| Yaw trim / target position / robot heading | Survey geometry; repeat from different headings and positions |
| Hood trim / nonlinear mapping | Measure multiple hood points on both sides |
| Random variation / camera noise | Re-annotate the same frames and compare independent measurements |
| Ball condition / mechanism changes | Identify balls and repeat controlled tests before and after each change |

Keep parameter values within the measured range. The dashboard accepts some broad mathematical bounds; those bounds are not physical evidence that a proposed value is right.

## 29. Final validation and what is still outside this model

Validate held-out stationary and moving shots using the saved configuration and actual loaded model. Cover both turrets, field positions/headings, speed directions, rotation signs, representative tilt, ball populations and normal wheel tracking. Check repeated makes, observed misses, rim contacts, flight timing and actual release-state residuals.

Compare fitted output errors against **measured** scoring margins where available. Distinguish a software evaluation test, a simulated collision-free score and a repeated physical score. The shipped baseline’s numerical report is not a hardware qualification.

Important current simplifications include a linear two-point hood mapping, one global speed-transfer ratio, one global spin ratio, constant effective aerodynamic coefficients, still air, a fixed nominal release origin/height, a simplified hub/rim, separate scalar release variabilities, and constant-motion release prediction. Hood-dependent compression, significant sidespin, flexible mechanism geometry, variable feed latency, strong acceleration and changing ball populations may require additional models rather than another scalar adjustment.

The app’s calibration controls do not change ferry behavior or feeding logic. Keep the existing manual controls for measured trials. Do not use an invalid targeting result as evidence that calibration should be invented or a mechanism protection should be removed.

## 30. Source and evidence notes

The factor names, baselines, units, supported controls and limitations in this guide were checked against the current `SimpleShotModel.java`, `PrecomputedShotMap.java`, `precomputed.ts`, `map-coverage.ts`, `physics.ts`, `types.ts` and log-import implementation in Rebuilt26. Known source values are not newly measured robot or field values.

External references support specific background statements only: [WPILib coordinate conventions](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html), [WPILib log timestamps](https://docs.wpilib.org/en/latest/docs/software/telemetry/datalog.html), [NASA drag equation](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/drag-equation/) and [NASA sphere drag dependence](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/shape-effects-on-drag/). The experiment design, worked examples and acceptance workflow are recommendations for this implementation, not published Chuck measurements.
