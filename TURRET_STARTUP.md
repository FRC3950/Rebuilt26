# Turret startup procedure

This branch requires physical placement and disabled dashboard confirmation before either turret
can track or spin its flywheels. The chosen center for **each turret is -180 degrees**, with a
**-189 to -171 degree** startup window. These angles are robot-relative, measured from that
turret's established shop zero along its negative-angle wire route. They are chosen operating poses,
not measurements inferred from the earlier sketch.

## Mark the robot once

1. Keep the robot disabled and safely support it. Establish each turret's existing zero mark and
   intended cable route. Do not change its Tuner X magnet offset.
2. Measure 180 degrees of negative turret rotation from that zero along the intended wire route.
   Check actual clearance and cable slack. Mark this as -180 degrees for each turret.
3. Mark boundaries 9 degrees on either side. Label the wire route as well: a pointing direction alone
   cannot distinguish different full-turn cable wraps.
4. If either pose is mechanically unsuitable, choose and measure a separate center for that turret
   and update `leftAzimuthStartupCenterDeg` or `rightAzimuthStartupCenterDeg` in `Constants.java`.
   Keep the entire window inside its configured control limits.

## Every robot-code startup or full power up

1. Stay **disabled**. Safely place both turrets inside their marked windows with the correct wire
   routes. Keep them stationary through confirmation. Do not enable the robot while hands are near it.
2. Run the SmartDashboard command **Turrets/Confirm both startup windows**.
3. Wait for **Turret19/StartupReady** and **Turret17/StartupReady** to both become true. Read each
   **StartupStatus** if initialization fails. **StartupAngleDeg** should match the physical angle
   inside its window. The software verifies CANcoder and Talon feedback within 0.5 degree and times
   out after 1 second following a successful position write; that does not independently verify
   physical placement. The write uses a 0.1-second Phoenix timeout on a background worker, keeping
   the command scheduler responsive. A rejected write shows its exact Phoenix status and description.
4. Clear people from the mechanism and enable. Tracking can move the turrets immediately. Initial
   testing should have no fuel loaded, a clear mechanism, and an operator ready to disable.

For hub tracking, while disabled run **Turrets/Use hub tracking**. Both **TargetingMode** displays
should read **Tracking target**. **Fixed angle (-135 deg)** means the operator A toggle has selected
the fixed-angle mode: in that mode the turrets do not follow the hub. The startup-window center
and this operating mode are separate settings. With normal default commands, tracking targets the
hub; held ferry-shot overrides can select a different target.

The action reads calibrated AbsolutePosition and adds the integer sensor turn nearest the center.
It assigns CANcoder **Position**, then checks both CANcoder Position and Talon selected feedback.
It does not change MagnetOffset or reset AbsolutePosition. No limit switches or previous-angle
storage are used. Initialization while enabled is ignored; enabling before verification completes
invalidates the attempt. A sensor reset or invalid feedback inhibits both turrets. A Talon reset
requires restarting robot code to reapply configuration before confirming the windows again.

If either turret is moved manually after confirmation, disable and repeat placement/confirmation
before tracking. Following any code restart, repeat the procedure even if CANcoders stayed powered.

## Validation on the robot

- At the center and both boundaries of each window, power-cycle, confirm while disabled, and check
  physical angle against StartupAngleDeg, CANcoder Position and Talon Position. At -180 degrees,
  selected sensor position should be approximately **-5 rotations** for the 10:1 reduction.
- Test small tracking moves after each initialization and inspect cable slack. Disable immediately
  if physical angle disagrees with measured angle.
- Start without confirmation: both azimuths and flywheels must remain stopped. Verify an enabled
  confirmation does not change CANcoder Position. Test enabling during verification and ensure both
  stay inhibited until a fresh disabled confirmation succeeds.
- While disabled, try just outside a window (for example -170 degrees). Confirmation should reject
  the reading. Return both turrets to their windows before retrying.
- With the robot disabled, disconnect/reconnect a CANcoder or reset a device in Tuner X. Verify
  readiness is lost and outputs stay inhibited. Follow the status message before reinitializing.
- Compare calibration/AbsolutePosition before and after assignment at the same physical pose.
  AbsolutePosition and MagnetOffset must be unchanged; only the continuous Position turn count changes.

**Physical placement is mandatory.** With a motor-side CANcoder and 10:1 reduction, positions 36
degrees apart give the same absolute reading. A turret at -144 degrees can be misinterpreted as
-180 degrees and still pass electronic verification. Do not deliberately enable from a wrong sector;
the marks and wire-route check are the only evidence of correct physical placement here.

Desktop simulation keeps its existing virtual startup reference and bypasses hardware initialization.
Unit tests exercise turn reconstruction, boundaries, discontinuity, illegal windows and the 36-degree
ambiguity. They do not validate the real CAN propagation or physical marks.

API references: [CTRE CANcoder](https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/hardware/core/CoreCANcoder.html),
[CTRE device resets](https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/hardware/ParentDevice.html),
[WPILib commands](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html).
