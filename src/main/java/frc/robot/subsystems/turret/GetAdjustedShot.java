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

public class GetAdjustedShot {
  private static GetAdjustedShot instance;

  private GetAdjustedShot() {}

  public static GetAdjustedShot getInstance() {
    if (instance == null) instance = new GetAdjustedShot();
    return instance;
  }

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

  private static final InterpolatingTreeMap<Double, Distancer> shotMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Distancer::interpolate);

  static {
    var rows = Distancer.loadRowsFromDeploy("shot_table.json");
    if (rows.isEmpty()) {
      minDistance = 0.0;
      maxDistance = 0.0;
      minTimeOfFlightSec = 0.0;
      maxTimeOfFlightSec = 0.0;
    } else {
      minDistance = rows.get(0).d;
      maxDistance = rows.get(rows.size() - 1).d;
      minTimeOfFlightSec = Double.POSITIVE_INFINITY;
      maxTimeOfFlightSec = Double.NEGATIVE_INFINITY;
    }

    for (var r : rows) {
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
    double robotAngle = robotPose.getRotation().getRadians();
    double turretVelocityX =
        fieldVelocity.vxMetersPerSecond
            + fieldVelocity.omegaRadiansPerSecond
                * (robotToTurret.getY() * Math.cos(robotAngle)
                    - robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        fieldVelocity.vyMetersPerSecond
            + fieldVelocity.omegaRadiansPerSecond
                * (robotToTurret.getX() * Math.cos(robotAngle)
                    - robotToTurret.getY() * Math.sin(robotAngle));

    // Time-of-flight from distance (plus extra latency)
    Distancer interpolatedForTof = shotMap.get(turretToTargetDistance);
    double timeOfFlight = interpolatedForTof.tofSec() + shotExtraLatencySec;

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
    Distancer interpolatedShot = shotMap.get(lookaheadTurretToTargetDistance);
    double hoodAngleDeg = interpolatedShot.hoodAngleDeg();

    // Velocity estimates (filtered)
    if (lastTurretAngle == null) lastTurretAngle = turretAngleRobot;

    double turretVelocity =
        turretAngleFilter.calculate(
            turretAngleRobot.minus(lastTurretAngle).getRadians() / Constants.loopPeriodSecs);

    lastTurretAngle = turretAngleRobot;

    boolean isDistanceValid =
        lookaheadTurretToTargetDistance >= minDistance
            && lookaheadTurretToTargetDistance <= maxDistance;
    String invalidReason =
        isDistanceValid
            ? ""
            : String.format(
                "lookahead distance %.2f m is outside shot table range [%.2f, %.2f] m",
                lookaheadTurretToTargetDistance, minDistance, maxDistance);

    ShootingParameters params =
        new ShootingParameters(
            isDistanceValid,
            turretAngleRobot,
            turretVelocity,
            hoodAngleDeg,
            interpolatedShot.flywheelRps(),
            invalidReason);
    return params;
  }

  public double getTimeOfFlight(double distance) {
    return shotMap.get(distance).tofSec();
  }

  public static double getMinTimeOfFlight() {
    return minTimeOfFlightSec + shotExtraLatencySec;
  }

  public static double getMaxTimeOfFlight() {
    return maxTimeOfFlightSec + shotExtraLatencySec;
  }

  public void clearShootingParameters() {
    // Shot parameters are recomputed on every request. This method remains as a no-op so
    // existing callers do not need to change.
  }
}
