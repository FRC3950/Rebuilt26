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
import frc.robot.Constants;
import frc.robot.util.Distancer;
import java.util.List;

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

  private static double minDistance;
  private static double maxDistance;
  private static final List<Distancer.Row> shotRows;

  private static final InterpolatingTreeMap<Double, Distancer> shotMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Distancer::interpolate);

  static {
    shotRows = Distancer.loadRowsFromDeploy("shot_table.json");
    if (shotRows.isEmpty()) {
      minDistance = 0.0;
      maxDistance = 0.0;
    } else {
      minDistance = shotRows.get(0).d;
      maxDistance = shotRows.get(shotRows.size() - 1).d;
    }

    for (var r : shotRows) {
      shotMap.put(r.d, new Distancer(r.hoodDeg, r.rps, r.tof));
    }
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

    Distancer initialShot = getShotForDistance(turretToTargetDistance);
    if (initialShot == null) {
      return new ShootingParameters(
          false, turretPosition.getRotation(), 0.0, 0.0, 0.0, "shot table is empty");
    }

    Translation2d turretVelocityField =
        getTurretFieldVelocity(robotPose, fieldVelocity, robotToTurret);
    Translation2d lookaheadTurretTranslation = turretPosition.getTranslation();
    double lookaheadDistance = turretToTargetDistance;

    for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
      Distancer shotForDistance = getShotForDistance(lookaheadDistance);
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

    Distancer interpolatedShot = getShotForDistance(lookaheadDistance);
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
