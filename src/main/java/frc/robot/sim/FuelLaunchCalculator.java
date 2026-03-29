package frc.robot.sim;

import static frc.robot.Constants.SimConstants.Fuel.EFFECTIVE_FLYWHEEL_RADIUS_METERS;
import static frc.robot.Constants.SimConstants.Fuel.EXIT_VELOCITY_SCALE;
import static frc.robot.Constants.SimConstants.Fuel.INTAKE_CENTER;
import static frc.robot.Constants.SimConstants.Fuel.TURRET_LAUNCH_HEIGHT_METERS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class FuelLaunchCalculator {
  public record LaunchState(Translation3d position, Translation3d velocity) {}

  public LaunchState calculateTurretLaunch(
      Pose2d robotPose,
      ChassisSpeeds fieldSpeeds,
      Translation2d robotToTurret,
      double turretYawDeg,
      double hoodAngleDeg,
      double flywheelRps) {
    Translation2d fieldOffset = robotToTurret.rotateBy(robotPose.getRotation());
    Translation2d fieldPosition = robotPose.getTranslation().plus(fieldOffset);
    double hoodRadians = Units.degreesToRadians(hoodAngleDeg);
    double fieldYawRadians =
        robotPose.getRotation().getRadians() + Units.degreesToRadians(turretYawDeg);
    double launchSpeedMetersPerSecond =
        2.0 * Math.PI * EFFECTIVE_FLYWHEEL_RADIUS_METERS * flywheelRps * EXIT_VELOCITY_SCALE;
    double horizontalSpeedMetersPerSecond = Math.cos(hoodRadians) * launchSpeedMetersPerSecond;

    return new LaunchState(
        new Translation3d(fieldPosition.getX(), fieldPosition.getY(), TURRET_LAUNCH_HEIGHT_METERS),
        new Translation3d(
            horizontalSpeedMetersPerSecond * Math.cos(fieldYawRadians)
                + fieldSpeeds.vxMetersPerSecond,
            horizontalSpeedMetersPerSecond * Math.sin(fieldYawRadians)
                + fieldSpeeds.vyMetersPerSecond,
            Math.sin(hoodRadians) * launchSpeedMetersPerSecond));
  }

  public LaunchState calculateOuttakeLaunch(
      Pose2d robotPose,
      ChassisSpeeds fieldSpeeds,
      double releaseHeightMeters,
      double outtakeSpeedMetersPerSecond) {
    Translation2d fieldOffset = INTAKE_CENTER.rotateBy(robotPose.getRotation());
    Translation2d fieldPosition = robotPose.getTranslation().plus(fieldOffset);
    double fieldYawRadians = robotPose.getRotation().getRadians();

    return new LaunchState(
        new Translation3d(fieldPosition.getX(), fieldPosition.getY(), releaseHeightMeters),
        new Translation3d(
            outtakeSpeedMetersPerSecond * Math.cos(fieldYawRadians) + fieldSpeeds.vxMetersPerSecond,
            outtakeSpeedMetersPerSecond * Math.sin(fieldYawRadians) + fieldSpeeds.vyMetersPerSecond,
            0.0));
  }
}
