package frc.robot.subsystems.turret;

import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Distancer;
import frc.robot.util.ShotLookup;
import frc.robot.util.ShotTableTarget;
import org.littletonrobotics.junction.Logger;

public class GetAdjustedShot {
  private static final double SHOT_EXTRA_LATENCY_SECS = 0.05;
  private static boolean lastReportedValid = true;
  private static String lastReportedInvalidReason = "";

  private double previousTangentialVelocityMetersPerSec = 0.0;
  private double previousTimestampSec = Double.NaN;

  private final ShotLookup hubShotLookup;
  private final ShotLookup ferryShotLookup;

  public GetAdjustedShot() {
    this(ShotLookup.fromDeploy(ShotTableTarget.HUB), ShotLookup.fromDeploy(ShotTableTarget.FERRY));
  }

  GetAdjustedShot(ShotLookup hubShotLookup, ShotLookup ferryShotLookup) {
    this.hubShotLookup = hubShotLookup;
    this.ferryShotLookup = ferryShotLookup;
  }

  public record ShootingParameters(
      boolean isValid,
      Rotation2d turretAngle, // robot-relative
      double turretVelocity, // rad/s (robot-relative)
      double hoodAngleDeg, // degrees
      double flywheelSpeed, // same units as flywheelSpeeds[] (ex: RPS)
      String invalidReason) {
    public boolean isValid() {
      if (isValid) {
        if (!lastReportedValid) {
          Logger.recordOutput("Turret/InvalidShotReason", "");
        }
        lastReportedValid = true;
      } else if (lastReportedValid || !invalidReason.equals(lastReportedInvalidReason)) {
        Logger.recordOutput("Turret/InvalidShotReason", invalidReason);
        lastReportedValid = false;
        lastReportedInvalidReason = invalidReason;
      }
      return isValid;
    }
  }

  public ShootingParameters getParameters(Pose2d robotPose, Translation2d robotToTurret) {
    return getParameters(robotPose, new ChassisSpeeds(), getHubTranslation(), robotToTurret, 0.0);
  }

  public ShootingParameters getParameters(
      Pose2d robotPose, Translation2d target, Translation2d robotToTurret) {
    return getParameters(robotPose, new ChassisSpeeds(), target, robotToTurret, 0.0);
  }

  public ShootingParameters getParameters(
      Pose2d robotPose, ChassisSpeeds fieldVelocity, Translation2d robotToTurret) {
    return getParameters(robotPose, fieldVelocity, getHubTranslation(), robotToTurret, 0.0);
  }

  public ShootingParameters getParameters(
      Pose2d robotPose,
      ChassisSpeeds fieldVelocity,
      Translation2d robotToTurret,
      double tofFudgeSec) {
    return getParameters(robotPose, fieldVelocity, getHubTranslation(), robotToTurret, tofFudgeSec);
  }

  public ShootingParameters getParameters(
      Pose2d robotPose,
      ChassisSpeeds fieldVelocity,
      Translation2d target,
      Translation2d robotToTurret) {
    return getParameters(robotPose, fieldVelocity, target, robotToTurret, 0.0);
  }

  public ShootingParameters getParameters(
      Pose2d robotPose,
      ChassisSpeeds fieldVelocity,
      Translation2d target,
      Translation2d robotToTurret,
      double tofFudgeSec) {
    return getParameters(
        robotPose, fieldVelocity, target, robotToTurret, tofFudgeSec, ShotTableTarget.HUB);
  }

  public ShootingParameters getParameters(
      Pose2d robotPose,
      ChassisSpeeds fieldVelocity,
      Translation2d target,
      Translation2d robotToTurret,
      double tofFudgeSec,
      ShotTableTarget shotTableTarget) {
    Pose2d turretPosition = robotPose.transformBy(new Transform2d(robotToTurret, Rotation2d.kZero));
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());
    Translation2d turretVelocityField =
        getTurretFieldVelocity(robotPose, fieldVelocity, robotToTurret);
    double radialVelocityMetersPerSec =
        getRadialComponent(target.minus(turretPosition.getTranslation()), turretVelocityField);

    Distancer interpolatedShot =
        getShotFor(shotTableTarget, turretToTargetDistance, radialVelocityMetersPerSec);
    if (interpolatedShot == null) {
      return new ShootingParameters(
          false,
          turretPosition.getRotation(),
          0.0,
          0.0,
          0.0,
          shotTableTarget.fileTargetName() + " shot table is empty");
    }

    double timeOfFlightSecs = getAdjustedTimeOfFlightSecs(interpolatedShot, tofFudgeSec);
    Translation2d lookaheadTurretTranslation =
        turretPosition
            .getTranslation()
            .plus(
                new Translation2d(
                    turretVelocityField.getX() * timeOfFlightSecs,
                    turretVelocityField.getY() * timeOfFlightSecs));

    Rotation2d turretAngleField = target.minus(lookaheadTurretTranslation).getAngle();
    Rotation2d turretAngleRobot = turretAngleField.minus(robotPose.getRotation());
    double turretVelocity =
        calculateTurretVelocity(
            target.minus(lookaheadTurretTranslation),
            turretVelocityField,
            fieldVelocity.omegaRadiansPerSecond,
            timeOfFlightSecs);

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

  private double calculateTurretVelocity(
      Translation2d turretToTarget,
      Translation2d turretVelocityField,
      double robotOmegaRadPerSec,
      double timeOfFlightSecs) {
    double distanceMeters = turretToTarget.getNorm();
    if (distanceMeters <= 0.0) {
      return 0.0;
    }

    double tangentialVelocityMetersPerSec =
        getTangentialComponent(turretToTarget, turretVelocityField);
    double tangentialAccelerationMetersPerSecSq =
        getTangentialAccelerationMetersPerSecSq(tangentialVelocityMetersPerSec);
    double angularVelocityFromTranslation = tangentialVelocityMetersPerSec / distanceMeters;
    double angularVelocityFromAcceleration =
        timeOfFlightSecs
            * tangentialAccelerationMetersPerSecSq
            * distanceMeters
            / (Math.pow(distanceMeters, 2)
                + Math.pow(tangentialVelocityMetersPerSec * timeOfFlightSecs, 2));

    return -robotOmegaRadPerSec + angularVelocityFromTranslation + angularVelocityFromAcceleration;
  }

  private double getTangentialAccelerationMetersPerSecSq(double tangentialVelocityMetersPerSec) {
    double nowSec = Timer.getFPGATimestamp();
    double accelerationMetersPerSecSq = 0.0;

    if (Double.isFinite(previousTimestampSec) && nowSec > previousTimestampSec) {
      accelerationMetersPerSecSq =
          (tangentialVelocityMetersPerSec - previousTangentialVelocityMetersPerSec)
              / (nowSec - previousTimestampSec);
    }

    previousTangentialVelocityMetersPerSec = tangentialVelocityMetersPerSec;
    previousTimestampSec = nowSec;
    return accelerationMetersPerSecSq;
  }

  private static double getTangentialComponent(Translation2d turretToTarget, Translation2d vector) {
    double distanceMeters = turretToTarget.getNorm();
    if (distanceMeters <= 0.0) {
      return 0.0;
    }

    Translation2d tangentDirection =
        turretToTarget.div(distanceMeters).rotateBy(Rotation2d.fromRadians(-Math.PI / 2.0));
    return vector.dot(tangentDirection);
  }

  private static double getRadialComponent(Translation2d turretToTarget, Translation2d vector) {
    double distanceMeters = turretToTarget.getNorm();
    if (distanceMeters <= 0.0) {
      return 0.0;
    }

    Translation2d targetDirection = turretToTarget.div(distanceMeters);
    return vector.dot(targetDirection);
  }

  private static double getAdjustedTimeOfFlightSecs(Distancer shot, double tofFudgeSec) {
    return Math.max(0.0, shot.tofSec() + SHOT_EXTRA_LATENCY_SECS + tofFudgeSec);
  }

  private Distancer getShotFor(
      ShotTableTarget target, double distanceMeters, double radialVelocityMetersPerSec) {
    ShotLookup shotLookup = target == ShotTableTarget.FERRY ? ferryShotLookup : hubShotLookup;
    if (shotLookup.isEmpty()) {
      return null;
    }
    if (target == ShotTableTarget.FERRY) {
      return shotLookup.getShotClamped(distanceMeters, radialVelocityMetersPerSec);
    }
    return shotLookup.getShot(distanceMeters, radialVelocityMetersPerSec);
  }
}
