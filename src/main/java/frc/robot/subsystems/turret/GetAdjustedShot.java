package frc.robot.subsystems.turret;

import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Distancer;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class GetAdjustedShot {
  private static final int LOOKAHEAD_ITERATIONS = 10;
  private static final double SHOT_EXTRA_LATENCY_SECS = 0.05;

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) Math.max(1, Math.round(0.1 / Constants.loopPeriodSecs)));
  private Rotation2d lastTurretAngle = null;

  public GetAdjustedShot() {}

  public record ShootingParameters(
      boolean isValid,
      Rotation2d turretAngle, // robot-relative
      double turretVelocity, // rad/s (robot-relative)
      double hoodAngleDeg, // degrees
      double flywheelSpeed, // same units as flywheelSpeeds[] (ex: RPS)
      String invalidReason) {
    public boolean isValid() {
      if (!isValid) {
        System.out.println("Invalid shot parameters: " + invalidReason);
      }
      return isValid;
    }
  }

  private static final String SHOT_TABLE_FILE = "shot_table.json";
  // Constant-arc table built in sim (see shot_table_analysis/). Only used while Peak Mode is on.
  private static final String PEAK_SHOT_TABLE_FILE = "shot_table_peak.json";
  private static final String PEAK_MODE_KEY = "Peak Mode";
  // A tuned row replaces any existing row this close to it.
  private static final double SAME_ROW_TOLERANCE_METERS = 0.1;

  private static final List<Distancer.Row> shotRows = new ArrayList<>();
  private static InterpolatingTreeMap<Double, Distancer> shotMap;

  private static final List<Distancer.Row> peakRows = new ArrayList<>();
  private static InterpolatingTreeMap<Double, Distancer> peakShotMap;

  static {
    shotRows.addAll(Distancer.loadRowsFromDeploy(SHOT_TABLE_FILE));
    rebuildShotMap();
    peakRows.addAll(Distancer.loadRowsFromDeploy(PEAK_SHOT_TABLE_FILE));
    peakShotMap = buildShotMap(peakRows);
  }

  private static void rebuildShotMap() {
    shotMap = buildShotMap(shotRows);
  }

  private static InterpolatingTreeMap<Double, Distancer> buildShotMap(List<Distancer.Row> rows) {
    rows.sort(Comparator.comparingDouble(r -> r.d));
    InterpolatingTreeMap<Double, Distancer> map =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Distancer::interpolate);
    for (var r : rows) {
      map.put(r.d, new Distancer(r.hoodDeg, r.rps, r.tof));
    }
    return map;
  }

  /** Publishes the Peak Mode switch, always off at startup so it never resumes on its own. */
  public static void initializeDashboard() {
    SmartDashboard.putBoolean(PEAK_MODE_KEY, false);
  }

  /** True when the driver enabled Peak Mode and the peak table actually loaded. */
  public static boolean isPeakModeActive() {
    return SmartDashboard.getBoolean(PEAK_MODE_KEY, false) && !peakRows.isEmpty();
  }

  // Robot code can't write to the deploy directory on the RIO, so tuned tables go to
  // /home/lvuser/shot_table.json there. In sim they go straight into src/main/deploy.
  private static File getTunedTableFile() {
    return Constants.currentMode == Constants.Mode.REAL
        ? new File(Filesystem.getOperatingDirectory(), SHOT_TABLE_FILE)
        : new File(Filesystem.getDeployDirectory(), SHOT_TABLE_FILE);
  }

  /**
   * Adds a tuned row, replacing any row within 10 cm, and uses it immediately. Time of flight is
   * carried over from the current table. Writes the whole table out and returns the file; the
   * in-memory table is updated even if the write throws.
   */
  public static File saveTunedRow(double distanceMeters, double hoodDeg, double flywheelRps)
      throws IOException {
    Distancer current = getShotForDistance(distanceMeters);

    Distancer.Row row = new Distancer.Row();
    row.d = Math.round(distanceMeters * 100.0) / 100.0;
    row.hoodDeg = hoodDeg;
    row.rps = flywheelRps;
    row.tof = current != null ? Math.round(current.tofSec() * 1000.0) / 1000.0 : 0.0;

    shotRows.removeIf(r -> Math.abs(r.d - row.d) < SAME_ROW_TOLERANCE_METERS);
    shotRows.add(row);
    rebuildShotMap();

    System.out.printf(
        "[ShotTable] {\"d\": %.2f, \"hoodDeg\": %.2f, \"rps\": %.2f, \"tof\": %.3f}%n",
        row.d, row.hoodDeg, row.rps, row.tof);
    File file = getTunedTableFile();
    Distancer.saveRows(file, shotRows);
    return file;
  }

  public ShootingParameters getParameters(Pose2d robotPose, Translation2d robotToTurret) {
    return getParameters(robotPose, new ChassisSpeeds(), getHubTranslation(), robotToTurret);
  }

  public ShootingParameters getParameters(
      Pose2d robotPose, Translation2d target, Translation2d robotToTurret) {
    return getParameters(robotPose, new ChassisSpeeds(), target, robotToTurret);
  }

  public ShootingParameters getParameters(
      Pose2d robotPose, ChassisSpeeds fieldVelocity, Translation2d robotToTurret) {
    return getParameters(robotPose, fieldVelocity, getHubTranslation(), robotToTurret);
  }

  public ShootingParameters getParameters(
      Pose2d robotPose,
      ChassisSpeeds fieldVelocity,
      Translation2d target,
      Translation2d robotToTurret) {
    Pose2d turretPosition = robotPose.transformBy(new Transform2d(robotToTurret, Rotation2d.kZero));
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Read the switch once per call so one shot never mixes rows from both tables.
    boolean peak = isPeakModeActive();
    Logger.recordOutput("ShotTable/PeakModeActive", peak);

    Distancer initialShot = getShotForDistance(turretToTargetDistance, peak);
    if (initialShot == null) {
      return new ShootingParameters(
          false, turretPosition.getRotation(), 0.0, 0.0, 0.0, "shot table is empty");
    }

    Translation2d turretVelocityField =
        getTurretFieldVelocity(robotPose, fieldVelocity, robotToTurret);
    Translation2d lookaheadTurretTranslation = turretPosition.getTranslation();
    double lookaheadDistance = turretToTargetDistance;

    for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
      Distancer shotForDistance = getShotForDistance(lookaheadDistance, peak);
      if (shotForDistance == null) {
        return new ShootingParameters(
            false, turretPosition.getRotation(), 0.0, 0.0, 0.0, "shot table is empty");
      }

      double timeOfFlightSecs = shotForDistance.tofSec() + SHOT_EXTRA_LATENCY_SECS;
      lookaheadTurretTranslation =
          turretPosition
              .getTranslation()
              .plus(
                  new Translation2d(
                      turretVelocityField.getX() * timeOfFlightSecs,
                      turretVelocityField.getY() * timeOfFlightSecs));
      lookaheadDistance = target.getDistance(lookaheadTurretTranslation);
    }

    Distancer interpolatedShot = getShotForDistance(lookaheadDistance, peak);
    if (interpolatedShot == null) {
      return new ShootingParameters(
          false, turretPosition.getRotation(), 0.0, 0.0, 0.0, "shot table is empty");
    }

    Rotation2d turretAngleField = target.minus(lookaheadTurretTranslation).getAngle();
    Rotation2d turretAngleRobot = turretAngleField.minus(robotPose.getRotation());
    double turretVelocity = calculateTurretVelocity(turretAngleRobot);

    return new ShootingParameters(
        true,
        turretAngleRobot,
        turretVelocity,
        interpolatedShot.hoodAngleDeg(),
        interpolatedShot.flywheelRps(),
        "");
  }

  private Translation2d getTurretFieldVelocity(
      Pose2d robotPose, ChassisSpeeds fieldVelocity, Translation2d robotToTurret) {
    Translation2d turretOffsetField = robotToTurret.rotateBy(robotPose.getRotation());
    double rotationalVelocityX = -fieldVelocity.omegaRadiansPerSecond * turretOffsetField.getY();
    double rotationalVelocityY = fieldVelocity.omegaRadiansPerSecond * turretOffsetField.getX();

    return new Translation2d(
        fieldVelocity.vxMetersPerSecond + rotationalVelocityX,
        fieldVelocity.vyMetersPerSecond + rotationalVelocityY);
  }

  private double calculateTurretVelocity(Rotation2d turretAngleRobot) {
    if (lastTurretAngle == null) {
      lastTurretAngle = turretAngleRobot;
    }

    double turretVelocity =
        turretAngleFilter.calculate(
            turretAngleRobot.minus(lastTurretAngle).getRadians() / Constants.loopPeriodSecs);
    lastTurretAngle = turretAngleRobot;
    return turretVelocity;
  }

  /** Returns the active table's hood/flywheel/tof for a distance, or null if the table is empty. */
  public static Distancer getTableShot(double distanceMeters) {
    return getShotForDistance(distanceMeters, isPeakModeActive());
  }

  private static Distancer getShotForDistance(double distance) {
    return getShotForDistance(distance, false);
  }

  private static Distancer getShotForDistance(double distance, boolean peak) {
    return peak
        ? getShotForDistance(peakRows, peakShotMap, distance)
        : getShotForDistance(shotRows, shotMap, distance);
  }

  private static Distancer getShotForDistance(
      List<Distancer.Row> rows, InterpolatingTreeMap<Double, Distancer> map, double distance) {
    if (rows.isEmpty()) {
      return null;
    }
    if (rows.size() == 1) {
      return Distancer.fromRow(rows.get(0));
    }
    if (distance < rows.get(0).d) {
      return interpolateBetweenRows(rows.get(0), rows.get(1), distance);
    }
    if (distance > rows.get(rows.size() - 1).d) {
      return interpolateBetweenRows(rows.get(rows.size() - 2), rows.get(rows.size() - 1), distance);
    }
    return map.get(distance);
  }

  private static Distancer interpolateBetweenRows(
      Distancer.Row start, Distancer.Row end, double distance) {
    double t = (distance - start.d) / (end.d - start.d);
    return Distancer.fromRow(start).interpolate(Distancer.fromRow(end), t);
  }
}
