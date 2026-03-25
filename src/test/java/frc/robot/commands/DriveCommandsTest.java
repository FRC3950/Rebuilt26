package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.hubTranslation;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class DriveCommandsTest {
  @Test
  void hubLineTargetPosePreservesXAndSnapsYToHub() {
    Pose2d currentPose = new Pose2d(3.2, 1.4, Rotation2d.fromDegrees(20.0));

    Pose2d targetPose = DriveCommands.getHubLineTargetPose(currentPose);

    assertEquals(currentPose.getX(), targetPose.getX(), 1e-9);
    assertEquals(hubTranslation.getY(), targetPose.getY(), 1e-9);
  }

  @Test
  void backFacingHeadingPointsRearTowardHub() {
    Translation2d alignedTranslation = new Translation2d(3.0, hubTranslation.getY());
    Pose2d fallbackPose = new Pose2d(alignedTranslation, Rotation2d.fromDegrees(15.0));

    Rotation2d heading = DriveCommands.getBackFacingHeading(alignedTranslation, fallbackPose);

    assertEquals(180.0, heading.getDegrees(), 1e-9);
  }
}
