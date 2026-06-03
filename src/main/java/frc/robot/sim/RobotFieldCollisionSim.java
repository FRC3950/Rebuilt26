package frc.robot.sim;

import static frc.robot.Constants.SimConstants.Fuel.ROBOT_LENGTH_METERS;
import static frc.robot.Constants.SimConstants.Fuel.ROBOT_WIDTH_METERS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class RobotFieldCollisionSim {
  private static final int MAX_SOLVER_ITERATIONS = 4;
  private static final double HALF_ROBOT_LENGTH = ROBOT_LENGTH_METERS / 2.0;
  private static final double HALF_ROBOT_WIDTH = ROBOT_WIDTH_METERS / 2.0;
  private static final Obstacle[] TRENCH_SIDE_OBSTACLES = {
    lowerTrenchSideObstacle(FuelSim.BLUE_BUMP_MIN_X, FuelSim.BLUE_BUMP_MAX_X),
    upperTrenchSideObstacle(FuelSim.BLUE_BUMP_MIN_X, FuelSim.BLUE_BUMP_MAX_X),
    lowerTrenchSideObstacle(FuelSim.RED_BUMP_MIN_X, FuelSim.RED_BUMP_MAX_X),
    upperTrenchSideObstacle(FuelSim.RED_BUMP_MIN_X, FuelSim.RED_BUMP_MAX_X)
  };
  private static final Obstacle[] FIELD_OBSTACLES = {
    hubObstacle(FuelSim.Hub.BLUE_HUB),
    hubObstacle(FuelSim.Hub.RED_HUB),
    TRENCH_SIDE_OBSTACLES[0],
    TRENCH_SIDE_OBSTACLES[1],
    TRENCH_SIDE_OBSTACLES[2],
    TRENCH_SIDE_OBSTACLES[3]
  };

  private RobotFieldCollisionSim() {}

  public static Pose2d constrainRobotPose(Pose2d pose) {
    Translation2d translation = constrainToFieldWalls(pose, pose.getTranslation());
    for (int i = 0; i < MAX_SOLVER_ITERATIONS; i++) {
      Translation2d before = translation;
      for (Obstacle obstacle : FIELD_OBSTACLES) {
        translation = obstacle.resolveCollision(pose, translation);
      }
      translation = constrainToFieldWalls(pose, translation);
      if (translation.getDistance(before) < 1e-9) {
        break;
      }
    }
    return new Pose2d(translation, pose.getRotation());
  }

  public static int getTrenchSideObstacleCountForSim() {
    return TRENCH_SIDE_OBSTACLES.length;
  }

  public static Pose2d getTrenchSideInteriorPoseForSim(int index) {
    Obstacle obstacle = getTrenchSideObstacleForSim(index);
    return new Pose2d(
        (obstacle.minX() + obstacle.maxX()) / 2.0,
        (obstacle.minY() + obstacle.maxY()) / 2.0,
        Rotation2d.kZero);
  }

  public static Pose2d getTrenchSideApproachPoseForSim(int index) {
    Obstacle obstacle = getTrenchSideObstacleForSim(index);
    double x = (obstacle.minX() + obstacle.maxX()) / 2.0;
    double y =
        isLowerTrenchSide(obstacle)
            ? obstacle.maxY() + HALF_ROBOT_WIDTH + 0.10
            : obstacle.minY() - HALF_ROBOT_WIDTH - 0.10;
    return new Pose2d(x, y, Rotation2d.kZero);
  }

  public static double getTrenchSideDriveYDirectionForSim(int index) {
    return isLowerTrenchSide(getTrenchSideObstacleForSim(index)) ? -1.0 : 1.0;
  }

  public static boolean robotFootprintClearsTrenchSideForSim(int index, Pose2d robotPose) {
    return getTrenchSideObstacleForSim(index).clearsRobotFootprint(robotPose);
  }

  public static boolean robotRemainsOnTrenchSideApproachSideForSim(int index, Pose2d robotPose) {
    Obstacle obstacle = getTrenchSideObstacleForSim(index);
    return obstacle.isRobotOnApproachSide(robotPose);
  }

  private static Obstacle getTrenchSideObstacleForSim(int index) {
    if (index < 0 || index >= TRENCH_SIDE_OBSTACLES.length) {
      throw new IllegalArgumentException("Invalid trench side obstacle index: " + index);
    }
    return TRENCH_SIDE_OBSTACLES[index];
  }

  private static boolean isLowerTrenchSide(Obstacle obstacle) {
    return (obstacle.minY() + obstacle.maxY()) / 2.0 < FuelSim.FIELD_WIDTH / 2.0;
  }

  private static Translation2d constrainToFieldWalls(Pose2d pose, Translation2d translation) {
    FootprintExtents extents = getFootprintExtents(pose);
    double constrainedX =
        Math.max(
            extents.xRadius(),
            Math.min(FuelSim.FIELD_LENGTH - extents.xRadius(), translation.getX()));
    double constrainedY =
        Math.max(
            extents.yRadius(),
            Math.min(FuelSim.FIELD_WIDTH - extents.yRadius(), translation.getY()));
    return new Translation2d(constrainedX, constrainedY);
  }

  private static FootprintExtents getFootprintExtents(Pose2d pose) {
    double cos = Math.abs(pose.getRotation().getCos());
    double sin = Math.abs(pose.getRotation().getSin());
    return new FootprintExtents(
        HALF_ROBOT_LENGTH * cos + HALF_ROBOT_WIDTH * sin,
        HALF_ROBOT_LENGTH * sin + HALF_ROBOT_WIDTH * cos);
  }

  private static Obstacle hubObstacle(FuelSim.Hub hub) {
    double halfSide = FuelSim.Hub.SIDE / 2.0;
    return new Obstacle(
        hub.center.getX() - halfSide,
        hub.center.getY() - halfSide,
        hub.center.getX() + halfSide,
        hub.center.getY() + halfSide);
  }

  private static Obstacle lowerTrenchSideObstacle(double minX, double maxX) {
    return new Obstacle(
        minX, FuelSim.LOWER_TRENCH_SIDE_MIN_Y, maxX, FuelSim.LOWER_TRENCH_SIDE_MAX_Y);
  }

  private static Obstacle upperTrenchSideObstacle(double minX, double maxX) {
    return new Obstacle(
        minX, FuelSim.UPPER_TRENCH_SIDE_MIN_Y, maxX, FuelSim.UPPER_TRENCH_SIDE_MAX_Y);
  }

  private record FootprintExtents(double xRadius, double yRadius) {}

  private record Obstacle(double minX, double minY, double maxX, double maxY) {
    private Translation2d resolveCollision(Pose2d pose, Translation2d robotCenter) {
      FootprintExtents extents = getFootprintExtents(new Pose2d(robotCenter, pose.getRotation()));
      double robotMinX = robotCenter.getX() - extents.xRadius();
      double robotMaxX = robotCenter.getX() + extents.xRadius();
      double robotMinY = robotCenter.getY() - extents.yRadius();
      double robotMaxY = robotCenter.getY() + extents.yRadius();

      if (robotMaxX <= minX || robotMinX >= maxX || robotMaxY <= minY || robotMinY >= maxY) {
        return robotCenter;
      }

      double pushLeft = robotMaxX - minX;
      double pushRight = maxX - robotMinX;
      double pushDown = robotMaxY - minY;
      double pushUp = maxY - robotMinY;
      double minPush = Math.min(Math.min(pushLeft, pushRight), Math.min(pushDown, pushUp));

      if (minPush == pushLeft) {
        return robotCenter.plus(new Translation2d(-pushLeft, 0.0));
      } else if (minPush == pushRight) {
        return robotCenter.plus(new Translation2d(pushRight, 0.0));
      } else if (minPush == pushDown) {
        return robotCenter.plus(new Translation2d(0.0, -pushDown));
      }
      return robotCenter.plus(new Translation2d(0.0, pushUp));
    }

    private boolean clearsRobotFootprint(Pose2d pose) {
      FootprintExtents extents = getFootprintExtents(pose);
      double robotMinX = pose.getX() - extents.xRadius();
      double robotMaxX = pose.getX() + extents.xRadius();
      double robotMinY = pose.getY() - extents.yRadius();
      double robotMaxY = pose.getY() + extents.yRadius();
      return robotMaxX <= minX || robotMinX >= maxX || robotMaxY <= minY || robotMinY >= maxY;
    }

    private boolean isRobotOnApproachSide(Pose2d pose) {
      FootprintExtents extents = getFootprintExtents(pose);
      if (isLowerTrenchSide(this)) {
        return pose.getY() - extents.yRadius() >= maxY;
      }
      return pose.getY() + extents.yRadius() <= minY;
    }
  }
}
