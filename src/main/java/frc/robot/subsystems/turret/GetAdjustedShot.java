package frc.robot.subsystems.turret;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.SubsystemConstants.Turret.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.util.Distancer;
import java.util.List;

public class GetAdjustedShot {
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

  // Filters (match MA style: moving average of velocity estimates)
  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

  private Rotation2d lastTurretAngle = null;

  private static double minDistance;
  private static double maxDistance;
  private static double minTimeOfFlightSec;
  private static double maxTimeOfFlightSec;
  private static final List<Distancer.Row> shotRows;

  private static final InterpolatingTreeMap<Double, Distancer> shotMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Distancer::interpolate);

  static {
    shotRows = Distancer.loadRowsFromDeploy("shot_table.json");
    if (shotRows.isEmpty()) {
      minDistance = 0.0;
      maxDistance = 0.0;
      minTimeOfFlightSec = 0.0;
      maxTimeOfFlightSec = 0.0;
    } else {
      minDistance = shotRows.get(0).d;
      maxDistance = shotRows.get(shotRows.size() - 1).d;
      minTimeOfFlightSec = Double.POSITIVE_INFINITY;
      maxTimeOfFlightSec = Double.NEGATIVE_INFINITY;
    }

    for (var r : shotRows) {
      shotMap.put(r.d, new Distancer(r.hoodDeg, r.rps, r.tof));
      minTimeOfFlightSec = Math.min(minTimeOfFlightSec, r.tof);
      maxTimeOfFlightSec = Math.max(maxTimeOfFlightSec, r.tof);
    }
  }

  public ShootingParameters getParameters(
      Pose2d robotPose, ChassisSpeeds fieldVelocity, Translation2d robotToTurret) {
    return getParameters(robotPose, fieldVelocity, hubTranslation, robotToTurret);
  }

  public ShootingParameters getParameters(
      Pose2d robotPose,
      ChassisSpeeds fieldVelocity,
      Translation2d target,
      Translation2d robotToTurret) {
    // Turret position on the field (robot pose + turret offset)
    Pose2d turretPosition = robotPose.transformBy(new Transform2d(robotToTurret, new Rotation2d()));
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Turret point velocity in FIELD coordinates (robot linear + omega cross r)
    Translation2d turretOffsetField = robotToTurret.rotateBy(robotPose.getRotation());
    double turretVelocityX =
        fieldVelocity.vxMetersPerSecond
            - fieldVelocity.omegaRadiansPerSecond * turretOffsetField.getY();
    double turretVelocityY =
        fieldVelocity.vyMetersPerSecond
            + fieldVelocity.omegaRadiansPerSecond * turretOffsetField.getX();

    // Time-of-flight from distance
    Distancer interpolatedForTof = getShotForDistance(turretToTargetDistance);
    if (interpolatedForTof == null) {
      return new ShootingParameters(
          false, turretPosition.getRotation(), 0.0, 0.0, 0.0, "shot table is empty");
    }
    double timeOfFlight = interpolatedForTof.tofSec();

    // Lookahead turret position (where the turret will be when the shot lands,
    // assuming constant velocity)
    double offsetX = turretVelocityX * timeOfFlight;
    double offsetY = turretVelocityY * timeOfFlight;
    Pose2d lookaheadPose =
        new Pose2d(
            turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
            turretPosition.getRotation());
    double lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());

    // Aim at target from lookahead position (FIELD), then convert to ROBOT-relative
    // for your turret setpoint
    Rotation2d turretAngleField = target.minus(lookaheadPose.getTranslation()).getAngle();
    Rotation2d turretAngleRobot = turretAngleField.minus(robotPose.getRotation());

    // Hood & flywheel from lookahead distance
    Distancer interpolatedShot = getShotForDistance(lookaheadTurretToTargetDistance);
    double hoodAngleDeg = interpolatedShot.hoodAngleDeg();

    // Velocity estimates (filtered)
    if (lastTurretAngle == null) lastTurretAngle = turretAngleRobot;

    double turretVelocity =
        turretAngleFilter.calculate(
            turretAngleRobot.minus(lastTurretAngle).getRadians() / Constants.loopPeriodSecs);

    lastTurretAngle = turretAngleRobot;

    ShootingParameters params =
        new ShootingParameters(
            true,
            turretAngleRobot,
            turretVelocity,
            hoodAngleDeg,
            interpolatedShot.flywheelRps(),
            "");
    return params;
  }

  public double getTimeOfFlight(double distance) {
    Distancer shot = getShotForDistance(distance);
    return shot == null ? 0.0 : shot.tofSec();
  }

  public static double getMinTimeOfFlight() {
    return minTimeOfFlightSec;
  }

  public static double getMaxTimeOfFlight() {
    return maxTimeOfFlightSec;
  }

  private static Distancer getShotForDistance(double distance) {
    if (shotRows.isEmpty()) {
      return null;
    }
    if (shotRows.size() == 1) {
      return Distancer.fromRow(shotRows.get(0));
    }
    if (distance < minDistance) {
      return interpolateBetweenRows(shotRows.get(0), shotRows.get(1), distance);
    }
    if (distance > maxDistance) {
      return interpolateBetweenRows(
          shotRows.get(shotRows.size() - 2), shotRows.get(shotRows.size() - 1), distance);
    }
    return shotMap.get(distance);
  }

  private static Distancer interpolateBetweenRows(
      Distancer.Row start, Distancer.Row end, double distance) {
    double t = (distance - start.d) / (end.d - start.d);
    return Distancer.fromRow(start).interpolate(Distancer.fromRow(end), t);
  }
}
