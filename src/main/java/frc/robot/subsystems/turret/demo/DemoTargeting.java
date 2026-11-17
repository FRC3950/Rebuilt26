package frc.robot.subsystems.turret.demo;

import static frc.robot.Constants.DemoConstants.*;
import static frc.robot.Constants.SubsystemConstants.Turret.maxFlywheelRps;
import static frc.robot.Constants.SubsystemConstants.Turret.maxHoodAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.minHoodAngle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.GetAdjustedShot;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.Distancer;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Aims one turret at the demo aim point and solves hood + flywheel physically for its distance and
 * height, with shoot-on-the-move lookahead. Everything is in the robot frame; field odometry is not
 * used. Only this turret is stopped when there is no target or no reachable shot.
 */
class DemoTargeting extends Command {
  private static final int LOOKAHEAD_ITERATIONS = 10;
  private static final Pose2d ROBOT_ORIGIN = Pose2d.kZero;
  static final String SPEED_TRIM_KEY = "Demo/Speed Trim";
  static final String HOOD_OVERRIDE_KEY = "Demo/Hood Override Deg";

  private final Turret turret;
  private final Drive drive;
  private final DemoTargetTracker tracker;
  private final BallisticShotModel model;
  private final Translation2d robotToTurret;

  DemoTargeting(
      Turret turret,
      Drive drive,
      DemoTargetTracker tracker,
      BallisticShotModel model,
      Translation2d robotToTurret) {
    this.turret = turret;
    this.drive = drive;
    this.tracker = tracker;
    this.model = model;
    this.robotToTurret = robotToTurret;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    String prefix = "Demo/" + turret.getName() + "/";
    Optional<Translation3d> aim = tracker.getAimPointRobotRelative();
    if (aim.isEmpty()) {
      SmartDashboard.putString(prefix + "Status", "No target - show tag " + tracker.getTagId());
      turret.stop();
      return;
    }
    Translation2d aimXY = aim.get().toTranslation2d();
    double dz = aim.get().getZ() - LAUNCH_HEIGHT_METERS;

    // Turret velocity in the robot's current frame (robot treated as sitting at the origin).
    Translation2d turretVelocity =
        GetAdjustedShot.getTurretFieldVelocity(
            ROBOT_ORIGIN, drive.getRobotRelativeSpeeds(), robotToTurret);
    double hoodOverride = SmartDashboard.getNumber(HOOD_OVERRIDE_KEY, 0.0);

    // Iterate: where will the turret be when the ball arrives, and what shot does that need?
    Translation2d lookahead = robotToTurret;
    BallisticShotModel.Solution solution = null;
    for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
      double d = aimXY.getDistance(lookahead);
      solution =
          model.solve(
              d,
              dz,
              minHoodAngle,
              maxHoodAngle,
              maxFlywheelRps,
              hoodOverride,
              preferredHoodDeg(d),
              HOOD_SEARCH_STEP_DEG);
      if (!solution.valid()) {
        break;
      }
      double t = solution.tofSec() + SHOT_EXTRA_LATENCY_SECS;
      lookahead = robotToTurret.plus(turretVelocity.times(t));
    }

    double distance = aimXY.getDistance(lookahead);
    SmartDashboard.putNumber(prefix + "Distance M", distance);
    SmartDashboard.putNumber(prefix + "Height Delta M", dz);
    if (solution == null || !solution.valid()) {
      String reason = solution == null ? "no solution" : solution.reason();
      SmartDashboard.putString(prefix + "Status", "No shot: " + reason);
      turret.stop();
      return;
    }

    double trim = SmartDashboard.getNumber(SPEED_TRIM_KEY, 1.0);
    double rps = Math.min(solution.flywheelRps() * trim, maxFlywheelRps);
    Rotation2d turretAngleRobot = aimXY.minus(lookahead).getAngle();

    turret.runSetpoints(turretAngleRobot, solution.hoodDeg(), rps);

    SmartDashboard.putString(prefix + "Status", "Tracking");
    SmartDashboard.putNumber(prefix + "Hood Deg", solution.hoodDeg());
    SmartDashboard.putNumber(prefix + "Flywheel RPS", rps);
    SmartDashboard.putNumber(prefix + "TOF Sec", solution.tofSec());
    Logger.recordOutput(prefix + "DistanceM", distance);
    Logger.recordOutput(prefix + "HoodDeg", solution.hoodDeg());
    Logger.recordOutput(prefix + "FlywheelRps", rps);
  }

  /**
   * Hood the hub table uses for a shot landing this far away. The table's distances are to hub
   * center while its shots land at the back of the funnel, so shift by that offset.
   */
  private static double preferredHoodDeg(double impactDistanceMeters) {
    Distancer tableShot =
        GetAdjustedShot.getTableShot(impactDistanceMeters - HUB_IMPACT_BEYOND_CENTER_METERS);
    return tableShot != null ? tableShot.hoodAngleDeg() : Double.NaN;
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
