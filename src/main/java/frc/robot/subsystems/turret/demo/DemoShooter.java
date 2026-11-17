package frc.robot.subsystems.turret.demo;

import static frc.robot.Constants.DemoConstants.*;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret1;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.Distancer;
import java.util.Optional;
import java.util.function.BooleanSupplier;

/**
 * Demo mode: turrets shoot at a point a set height above one AprilTag, solving hood and flywheel
 * for that distance AND height. Field odometry is not used, only the Limelights plus the robot's
 * own motion.
 *
 * <p>This subsystem tracks the tag every loop and hands out the turret targeting command via {@link
 * #targetingCommand}; RobotContainer picks it as each turret's default command in demo mode.
 *
 * <p>Dashboard controls include Tag ID, Height Above Tag M, Range Offset Ft, Speed Trim, Hood
 * Override Deg, and Clear Target.
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
    SmartDashboard.setDefaultNumber(DemoTargeting.RANGE_OFFSET_KEY, DEFAULT_RANGE_OFFSET_FEET);
    SmartDashboard.setDefaultNumber(DemoTargeting.SPEED_TRIM_KEY, DEFAULT_SPEED_TRIM);
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
    var hub = tracker.getAimPointRobotRelative().map(Translation3d::toTranslation2d);
    publishHubDistance("Demo/Hub Distance/Robot Ft", hub, Translation2d.kZero);
    publishHubDistance("Demo/Hub Distance/Left Turret Ft", hub, robotToTurret1);
    publishHubDistance("Demo/Hub Distance/Right Turret Ft", hub, robotToTurret2);
  }

  private static void publishHubDistance(
      String key, Optional<Translation2d> hub, Translation2d origin) {
    SmartDashboard.putNumber(
        key, hub.map(target -> Units.metersToFeet(target.getDistance(origin))).orElse(-1.0));
  }
}
