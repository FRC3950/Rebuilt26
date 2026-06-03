package frc.robot.sim;

import static frc.robot.Constants.SimConstants.Fuel.ROBOT_LENGTH_METERS;
import static frc.robot.Constants.SimConstants.Fuel.ROBOT_WIDTH_METERS;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class RobotFieldCollisionSimTest {
  private static final double EPSILON = 1e-9;

  @Test
  void constrainsRobotInsideFieldWalls() {
    Pose2d corrected =
        RobotFieldCollisionSim.constrainRobotPose(
            new Pose2d(-0.25, FuelSim.FIELD_WIDTH / 2.0, Rotation2d.kZero));

    assertEquals(ROBOT_LENGTH_METERS / 2.0, corrected.getX(), EPSILON);
    assertEquals(FuelSim.FIELD_WIDTH / 2.0, corrected.getY(), EPSILON);
    assertEquals(Rotation2d.kZero, corrected.getRotation());
  }

  @Test
  void pushesRobotOutOfHubFootprint() {
    Pose2d corrected =
        RobotFieldCollisionSim.constrainRobotPose(
            new Pose2d(
                FuelSim.Hub.BLUE_HUB.center.getX(),
                FuelSim.Hub.BLUE_HUB.center.getY(),
                Rotation2d.kZero));

    double distanceFromHubCenter =
        corrected.getTranslation().getDistance(FuelSim.Hub.BLUE_HUB.center);
    assertTrue(distanceFromHubCenter > 0.0);
    assertTrue(
        corrected.getX() <= FuelSim.Hub.BLUE_HUB.center.getX() - FuelSim.Hub.SIDE / 2.0
            || corrected.getX() >= FuelSim.Hub.BLUE_HUB.center.getX() + FuelSim.Hub.SIDE / 2.0
            || corrected.getY() <= FuelSim.Hub.BLUE_HUB.center.getY() - FuelSim.Hub.SIDE / 2.0
            || corrected.getY() >= FuelSim.Hub.BLUE_HUB.center.getY() + FuelSim.Hub.SIDE / 2.0);
  }

  @Test
  void bumpAreaRemainsTraversableInsteadOfBecomingHardCollision() {
    Pose2d bumpPose = new Pose2d(4.1, 2.5, Rotation2d.kZero);

    assertEquals(bumpPose, RobotFieldCollisionSim.constrainRobotPose(bumpPose));
  }

  @Test
  void trenchBarAreaRemainsTraversableBecauseRobotCanDriveUnderIt() {
    Pose2d underTrenchBarPose = new Pose2d(FuelSim.BLUE_BUMP_PEAK_X, 0.65, Rotation2d.kZero);

    assertEquals(underTrenchBarPose, RobotFieldCollisionSim.constrainRobotPose(underTrenchBarPose));
  }

  @Test
  void trenchSideBlockPushesRobotOutInsteadOfAllowingClippingThroughLowHardware() {
    assertTrenchSideCollisionPushesRobotClear(
        FuelSim.BLUE_BUMP_MIN_X,
        FuelSim.BLUE_BUMP_MAX_X,
        FuelSim.LOWER_TRENCH_SIDE_MIN_Y,
        FuelSim.LOWER_TRENCH_SIDE_MAX_Y);
    assertTrenchSideCollisionPushesRobotClear(
        FuelSim.RED_BUMP_MIN_X,
        FuelSim.RED_BUMP_MAX_X,
        FuelSim.LOWER_TRENCH_SIDE_MIN_Y,
        FuelSim.LOWER_TRENCH_SIDE_MAX_Y);
    assertTrenchSideCollisionPushesRobotClear(
        FuelSim.BLUE_BUMP_MIN_X,
        FuelSim.BLUE_BUMP_MAX_X,
        FuelSim.UPPER_TRENCH_SIDE_MIN_Y,
        FuelSim.UPPER_TRENCH_SIDE_MAX_Y);
    assertTrenchSideCollisionPushesRobotClear(
        FuelSim.RED_BUMP_MIN_X,
        FuelSim.RED_BUMP_MAX_X,
        FuelSim.UPPER_TRENCH_SIDE_MIN_Y,
        FuelSim.UPPER_TRENCH_SIDE_MAX_Y);
  }

  private static void assertTrenchSideCollisionPushesRobotClear(
      double minX, double maxX, double minY, double maxY) {
    Pose2d insideLowerTrenchSide =
        new Pose2d((minX + maxX) / 2.0, (minY + maxY) / 2.0, Rotation2d.kZero);

    Pose2d corrected = RobotFieldCollisionSim.constrainRobotPose(insideLowerTrenchSide);

    assertTrue(
        corrected.getTranslation().getDistance(insideLowerTrenchSide.getTranslation()) > 0.0);
    assertTrue(robotFootprintClearsObstacle(corrected, minX, maxX, minY, maxY));
  }

  private static boolean robotFootprintClearsObstacle(
      Pose2d robotPose, double minX, double maxX, double minY, double maxY) {
    double robotMinX = robotPose.getX() - ROBOT_LENGTH_METERS / 2.0;
    double robotMaxX = robotPose.getX() + ROBOT_LENGTH_METERS / 2.0;
    double robotMinY = robotPose.getY() - ROBOT_WIDTH_METERS / 2.0;
    double robotMaxY = robotPose.getY() + ROBOT_WIDTH_METERS / 2.0;
    return robotMaxX <= minX || robotMinX >= maxX || robotMaxY <= minY || robotMinY >= maxY;
  }
}
