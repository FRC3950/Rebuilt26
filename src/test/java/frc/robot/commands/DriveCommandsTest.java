package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.hubTranslation;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class DriveCommandsTest {
  @Test
  void hubLineTargetPosePreservesXAndSnapsYToHub() {
    Pose2d currentPose = new Pose2d(3.2, 1.4, Rotation2d.fromDegrees(20.0));

    Pose2d targetPose =
        DriveCommands.getHubLineTargetPose(
            currentPose, Constants.SubsystemConstants.Turret.robotToTurret1);

    assertEquals(currentPose.getX(), targetPose.getX(), 1e-9);
    Translation2d turretTranslation =
        targetPose
            .transformBy(
                new edu.wpi.first.math.geometry.Transform2d(
                    Constants.SubsystemConstants.Turret.robotToTurret1, Rotation2d.kZero))
            .getTranslation();
    assertEquals(hubTranslation.getY(), turretTranslation.getY(), 1e-9);
  }

  @Test
  void backFacingHeadingPointsRearTowardHub() {
    Translation2d alignedTranslation = new Translation2d(3.0, hubTranslation.getY());
    Pose2d fallbackPose = new Pose2d(alignedTranslation, Rotation2d.fromDegrees(15.0));

    Rotation2d heading = DriveCommands.getBackFacingHeading(alignedTranslation, fallbackPose);

    assertEquals(180.0, heading.getDegrees(), 1e-9);
  }

  @Test
  void turretDistanceUsesTurretOffsetNotRobotCenter() {
    Pose2d robotPose = new Pose2d(3.0, 4.0, Rotation2d.kZero);

    double distance =
        DriveCommands.getTurretDistanceToHubMeters(
            robotPose, Constants.SubsystemConstants.Turret.robotToTurret1);

    Translation2d turretTranslation =
        robotPose
            .transformBy(
                new edu.wpi.first.math.geometry.Transform2d(
                    Constants.SubsystemConstants.Turret.robotToTurret1, Rotation2d.kZero))
            .getTranslation();
    assertEquals(hubTranslation.getDistance(turretTranslation), distance, 1e-9);
  }

  @Test
  void distanceTargetPosePreservesHeading() {
    Pose2d robotPose = new Pose2d(6.0, 3.0, Rotation2d.fromDegrees(35.0));

    Pose2d targetPose =
        DriveCommands.getLeftTurretHubDistanceTargetPose(
            robotPose, Constants.SubsystemConstants.Turret.robotToTurret1, 2.5);

    assertEquals(robotPose.getRotation().getRadians(), targetPose.getRotation().getRadians(), 1e-9);
  }

  @Test
  void shorterDistanceMovesTowardHub() {
    Pose2d robotPose = new Pose2d(7.0, 4.0, Rotation2d.kZero);
    double currentDistance =
        DriveCommands.getTurretDistanceToHubMeters(
            robotPose, Constants.SubsystemConstants.Turret.robotToTurret1);

    Pose2d targetPose =
        DriveCommands.getLeftTurretHubDistanceTargetPose(
            robotPose, Constants.SubsystemConstants.Turret.robotToTurret1, currentDistance - 1.0);

    double targetDistance =
        DriveCommands.getTurretDistanceToHubMeters(
            targetPose, Constants.SubsystemConstants.Turret.robotToTurret1);
    assertTrue(targetDistance < currentDistance);
  }

  @Test
  void longerDistanceMovesAwayFromHub() {
    Pose2d robotPose = new Pose2d(7.0, 4.0, Rotation2d.kZero);
    double currentDistance =
        DriveCommands.getTurretDistanceToHubMeters(
            robotPose, Constants.SubsystemConstants.Turret.robotToTurret1);

    Pose2d targetPose =
        DriveCommands.getLeftTurretHubDistanceTargetPose(
            robotPose, Constants.SubsystemConstants.Turret.robotToTurret1, currentDistance + 1.0);

    double targetDistance =
        DriveCommands.getTurretDistanceToHubMeters(
            targetPose, Constants.SubsystemConstants.Turret.robotToTurret1);
    assertTrue(targetDistance > currentDistance);
  }

  @Test
  void zeroHubVectorFallsBackSafely() {
    Translation2d robotToTurret = Constants.SubsystemConstants.Turret.robotToTurret1;
    Pose2d robotPose =
        new Pose2d(
            hubTranslation.minus(robotToTurret.rotateBy(Rotation2d.kZero)), Rotation2d.kZero);

    Translation2d direction =
        DriveCommands.getHubDistanceTranslationDirection(robotPose, robotToTurret);

    assertEquals(1.0, direction.getNorm(), 1e-9);
  }

  @Test
  void distanceToleranceMatchesThreshold() {
    assertTrue(DriveCommands.isWithinDistanceTolerance(2.0, 2.04));
    assertFalse(DriveCommands.isWithinDistanceTolerance(2.0, 2.2));
  }
}
