package frc.robot.subsystems.turret;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.SubsystemConstants.Turret.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.util.Distancer;
import java.util.List;

public class GetAdjustedShot {
  public GetAdjustedShot() {}

  public record ShootingParameters(
      boolean isValid,
      Rotation2d turretAngle, // robot-relative
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
    return getParameters(robotPose, hubTranslation, robotToTurret);
  }

  public ShootingParameters getParameters(
      Pose2d robotPose, Translation2d target, Translation2d robotToTurret) {
    Pose2d turretPosition = robotPose.transformBy(new Transform2d(robotToTurret, Rotation2d.kZero));
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    Distancer interpolatedShot = getShotForDistance(turretToTargetDistance);
    if (interpolatedShot == null) {
      return new ShootingParameters(
          false, turretPosition.getRotation(), 0.0, 0.0, "shot table is empty");
    }

    Rotation2d turretAngleField = target.minus(turretPosition.getTranslation()).getAngle();
    Rotation2d turretAngleRobot = turretAngleField.minus(robotPose.getRotation());

    return new ShootingParameters(
        true,
        turretAngleRobot,
        interpolatedShot.hoodAngleDeg(),
        interpolatedShot.flywheelRps(),
        "");
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
