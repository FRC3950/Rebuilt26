package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import org.junit.jupiter.api.Test;

class RobotBumpSimTest {
  private static final double EPSILON = 1e-9;

  @Test
  void flatFieldUpdateReturnsFlatPoseAndNoRampOverride() {
    RobotBumpSim bumpSim = new RobotBumpSim(Drive.getModuleTranslations());

    var pose3d =
        bumpSim.update(
            new Pose2d(2.0, FuelSim.FIELD_WIDTH / 2.0, Rotation2d.kZero), new ChassisSpeeds(), 5);

    assertFalse(bumpSim.isOnRamp());
    assertEquals(0.0, pose3d.getZ(), EPSILON);
    assertEquals(0.0, pose3d.getRotation().getX(), EPSILON);
    assertEquals(0.0, pose3d.getRotation().getY(), EPSILON);
  }

  @Test
  void enteringBlueBumpCreatesMapleSimRampOverridePose() {
    var moduleOffsets = Drive.getModuleTranslations();
    RobotBumpSim bumpSim = new RobotBumpSim(moduleOffsets);
    Pose2d poseWithFrontModulesTouchingRamp =
        new Pose2d(
            FuelSim.BLUE_BUMP_MIN_X - moduleOffsets[0].getX() + 0.02,
            FuelSim.LOWER_BUMP_CENTER_Y,
            Rotation2d.kZero);

    var pose3d =
        bumpSim.update(poseWithFrontModulesTouchingRamp, new ChassisSpeeds(1.0, 0.0, 0.0), 5);
    Pose2d mapleOverridePose = bumpSim.getSimWorldPose(poseWithFrontModulesTouchingRamp);

    assertTrue(bumpSim.isOnRamp());
    assertTrue(pose3d.getZ() > 0.0);
    assertTrue(Math.abs(pose3d.getRotation().getY()) > 0.0);
    assertEquals(mapleOverridePose.getX(), pose3d.getX(), EPSILON);
    assertEquals(poseWithFrontModulesTouchingRamp.getY(), mapleOverridePose.getY(), EPSILON);
    assertEquals(poseWithFrontModulesTouchingRamp.getRotation(), mapleOverridePose.getRotation());
  }

  @Test
  void slowHeadOnBumpAttemptSlidesBackInsteadOfGhostingThroughPeak() {
    var moduleOffsets = Drive.getModuleTranslations();
    RobotBumpSim bumpSim = new RobotBumpSim(moduleOffsets);
    Pose2d pose =
        new Pose2d(
            FuelSim.BLUE_BUMP_MIN_X - moduleOffsets[0].getX() + 0.02,
            FuelSim.LOWER_BUMP_CENTER_Y,
            Rotation2d.kZero);
    double startingX = pose.getX();

    for (int i = 0; i < 80; i++) {
      bumpSim.update(pose, new ChassisSpeeds(0.2, 0.0, 0.0), 5);
      pose =
          bumpSim.isOnRamp()
              ? bumpSim.getSimWorldPose(pose)
              : new Pose2d(pose.getX() + 0.2 * FuelSim.PERIOD, pose.getY(), pose.getRotation());
    }

    assertTrue(pose.getX() < FuelSim.BLUE_BUMP_PEAK_X);
    assertTrue(pose.getX() <= startingX + 0.15);
  }

  @Test
  void nearThresholdHeadOnBumpAttemptIsSlowedByRampFriction() {
    var moduleOffsets = Drive.getModuleTranslations();
    RobotBumpSim bumpSim = new RobotBumpSim(moduleOffsets);
    Pose2d pose =
        new Pose2d(
            FuelSim.BLUE_BUMP_MIN_X - moduleOffsets[0].getX() + 0.02,
            FuelSim.LOWER_BUMP_CENTER_Y,
            Rotation2d.kZero);
    double startingX = pose.getX();

    for (int i = 0; i < 80; i++) {
      bumpSim.update(pose, new ChassisSpeeds(1.8, 0.0, 0.0), 5);
      pose =
          bumpSim.isOnRamp()
              ? bumpSim.getSimWorldPose(pose)
              : new Pose2d(pose.getX() + 1.8 * FuelSim.PERIOD, pose.getY(), pose.getRotation());
    }

    assertTrue(
        pose.getX() <= startingX + 1.0,
        "near-threshold bump crossing should lose some forward travel to ramp friction, x="
            + pose.getX());
  }
}
