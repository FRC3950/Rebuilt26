package frc.robot.subsystems.turret.demo;

import static frc.robot.Constants.DemoConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.Distancer;
import java.util.function.BooleanSupplier;

/**
 * Demo mode: turrets shoot at a point a set height above one AprilTag, solving hood and flywheel
 * for that distance AND height. Field odometry is not used, only the Limelights plus the robot's
 * own motion.
 *
 * <p>This subsystem tracks the tag every loop and hands out the turret targeting command via {@link
 * #targetingCommand}; RobotContainer picks it as each turret's default command in demo mode.
 *
 * <p>Dashboard (under "Demo/"): Tag ID, Height Above Tag M, Speed Trim, Hood Override Deg, Clear
 * Target, plus per-turret status/telemetry.
 */
public class DemoShooter extends SubsystemBase {
  private final Drive drive;
  private final BooleanSupplier demoSelected;
  private final DemoTargetTracker tracker;
  private final BallisticShotModel model;

  public DemoShooter(Drive drive, BooleanSupplier demoSelected) {
    this.drive = drive;
    this.demoSelected = demoSelected;
    tracker = new DemoTargetTracker(drive);

    // Calibrated from the hub table, whose shots land at the back of the HUB funnel.
    model =
        BallisticShotModel.fitFromShotTable(
            Distancer.loadRowsFromDeploy("shot_table.json"),
            LAUNCH_HEIGHT_METERS,
            HUB_IMPACT_HEIGHT_METERS,
            HUB_IMPACT_BEYOND_CENTER_METERS);

    SmartDashboard.putBoolean("Demo/Shot Model Calibrated", model.isCalibrated());
    // Multiplies the solved flywheel speed; raise above 1.0 if long shots fall short (drag).
    SmartDashboard.setDefaultNumber(DemoTargeting.SPEED_TRIM_KEY, 1.0);
    // Outside the hood range (e.g. 0) = auto.
    SmartDashboard.setDefaultNumber(DemoTargeting.HOOD_OVERRIDE_KEY, 0.0);
    SmartDashboard.putData(
        "Demo/Clear Target", Commands.runOnce(tracker::clearTarget).ignoringDisable(true));
  }

  /** Command that aims one turret at the demo target. Requires only that turret. */
  public Command targetingCommand(Turret turret, Translation2d robotToTurret) {
    return new DemoTargeting(turret, drive, tracker, model, robotToTurret);
  }

  @Override
  public void periodic() {
    tracker.update();
    SmartDashboard.putBoolean("Demo/Active", demoSelected.getAsBoolean());
  }
}
