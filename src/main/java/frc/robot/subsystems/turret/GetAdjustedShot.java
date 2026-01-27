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
      double hoodAngle, // radians
      double hoodVelocity, // rad/s
      double flywheelSpeed // same units as flywheelSpeeds[] (ex: RPS)
      ) {}

  private ShootingParameters latestParameters = null;

  // Filters (match MA style: moving average of velocity estimates)
  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

  private Rotation2d lastTurretAngle = null;
  private double lastHoodAngle = Double.NaN;

  private static double minDistance;
  private static double maxDistance;

  private static final InterpolatingTreeMap<Double, Distancer> shotMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Distancer::interpolate);

  static {
    var rows = Distancer.loadRowsFromDeploy("shot_table.json");
    for (var r : rows) {
      shotMap.put(r.d, new Distancer(Rotation2d.fromDegrees(r.hoodDeg), r.rps, r.tof));
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
    if (latestParameters != null && target.equals(hubTranslation)) {
      return latestParameters;
    }

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
    double hoodAngle = interpolatedShot.hoodAngle().getRadians();

    // Velocity estimates (filtered)
    if (lastTurretAngle == null) lastTurretAngle = turretAngleRobot;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

    double turretVelocity =
        turretAngleFilter.calculate(
            turretAngleRobot.minus(lastTurretAngle).getRadians() / Constants.loopPeriodSecs);
    double hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);

    lastTurretAngle = turretAngleRobot;
    lastHoodAngle = hoodAngle;

    ShootingParameters params =
        new ShootingParameters(
            lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance,
            turretAngleRobot,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            interpolatedShot.flywheelRps());

    if (target.equals(hubTranslation)) {
      latestParameters = params;
    }
    return params;
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }
}
