package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** MapleSim-compatible robot ramp model for the REBUILT field bumps. */
public final class RobotBumpSim {
  public static final int DEFAULT_SUBTICKS = 5;

  private static final double WHEEL_RADIUS_METERS = 0.048;
  private static final double CHASSIS_HEIGHT_METERS = 0.0;
  private static final double BUMP_COR = 0.15;
  private static final double SEGMENT_PROJECTION_TOLERANCE = 1e-6;

  private static final Translation3d[] BUMP_LINE_STARTS = {
    new Translation3d(FuelSim.BLUE_BUMP_MIN_X, FuelSim.LOWER_BUMP_MIN_Y, 0.0),
    new Translation3d(FuelSim.BLUE_BUMP_MIN_X, FuelSim.UPPER_BUMP_MIN_Y, 0.0),
    new Translation3d(FuelSim.BLUE_BUMP_PEAK_X, FuelSim.LOWER_BUMP_MIN_Y, FuelSim.BUMP_HEIGHT),
    new Translation3d(FuelSim.BLUE_BUMP_PEAK_X, FuelSim.UPPER_BUMP_MIN_Y, FuelSim.BUMP_HEIGHT),
    new Translation3d(FuelSim.RED_BUMP_MIN_X, FuelSim.LOWER_BUMP_MIN_Y, 0.0),
    new Translation3d(FuelSim.RED_BUMP_MIN_X, FuelSim.UPPER_BUMP_MIN_Y, 0.0),
    new Translation3d(FuelSim.RED_BUMP_PEAK_X, FuelSim.LOWER_BUMP_MIN_Y, FuelSim.BUMP_HEIGHT),
    new Translation3d(FuelSim.RED_BUMP_PEAK_X, FuelSim.UPPER_BUMP_MIN_Y, FuelSim.BUMP_HEIGHT)
  };

  private static final Translation3d[] BUMP_LINE_ENDS = {
    new Translation3d(FuelSim.BLUE_BUMP_PEAK_X, FuelSim.LOWER_BUMP_MAX_Y, FuelSim.BUMP_HEIGHT),
    new Translation3d(FuelSim.BLUE_BUMP_PEAK_X, FuelSim.UPPER_BUMP_MAX_Y, FuelSim.BUMP_HEIGHT),
    new Translation3d(FuelSim.BLUE_BUMP_MAX_X, FuelSim.LOWER_BUMP_MAX_Y, 0.0),
    new Translation3d(FuelSim.BLUE_BUMP_MAX_X, FuelSim.UPPER_BUMP_MAX_Y, 0.0),
    new Translation3d(FuelSim.RED_BUMP_PEAK_X, FuelSim.LOWER_BUMP_MAX_Y, FuelSim.BUMP_HEIGHT),
    new Translation3d(FuelSim.RED_BUMP_PEAK_X, FuelSim.UPPER_BUMP_MAX_Y, FuelSim.BUMP_HEIGHT),
    new Translation3d(FuelSim.RED_BUMP_MAX_X, FuelSim.LOWER_BUMP_MAX_Y, 0.0),
    new Translation3d(FuelSim.RED_BUMP_MAX_X, FuelSim.UPPER_BUMP_MAX_Y, 0.0)
  };

  private final Translation2d[] moduleOffsets;
  private final double[] moduleZPositions = new double[4];
  private final double[] moduleZVelocities = new double[4];
  private final double frontBackDistance;
  private final double leftRightDistance;

  private boolean onRamp = false;
  private double simXPosition = 0.0;
  private double simXVelocity = 0.0;

  public RobotBumpSim(Translation2d[] moduleOffsets) {
    if (moduleOffsets.length != 4) {
      throw new IllegalArgumentException("RobotBumpSim requires four module offsets");
    }
    this.moduleOffsets = moduleOffsets.clone();

    double frontX = (moduleOffsets[0].getX() + moduleOffsets[1].getX()) / 2.0;
    double backX = (moduleOffsets[2].getX() + moduleOffsets[3].getX()) / 2.0;
    frontBackDistance = Math.max(Math.abs(frontX - backX), 1e-3);

    double leftY = (moduleOffsets[0].getY() + moduleOffsets[2].getY()) / 2.0;
    double rightY = (moduleOffsets[1].getY() + moduleOffsets[3].getY()) / 2.0;
    leftRightDistance = Math.max(Math.abs(leftY - rightY), 1e-3);
  }

  public boolean isOnRamp() {
    return onRamp;
  }

  public Pose2d getSimWorldPose(Pose2d latestMaplePose) {
    return new Pose2d(simXPosition, latestMaplePose.getY(), latestMaplePose.getRotation());
  }

  public Pose3d update(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, int subticks) {
    int safeSubticks = Math.max(1, subticks);
    double dt = FuelSim.PERIOD / safeSubticks;
    double mapleVx = fieldRelativeSpeeds.vxMetersPerSecond;
    double contactFactor = Math.abs(Math.cos(2.0 * robotPose.getRotation().getRadians()));
    double[] worldY = new double[moduleOffsets.length];
    Translation2d[] rotatedModuleOffsets = new Translation2d[moduleOffsets.length];

    for (int i = 0; i < moduleOffsets.length; i++) {
      rotatedModuleOffsets[i] = moduleOffsets[i].rotateBy(robotPose.getRotation());
      worldY[i] = robotPose.getY() + rotatedModuleOffsets[i].getY();
    }

    for (int tick = 0; tick < safeSubticks; tick++) {
      double currentRobotX = onRamp ? simXPosition : robotPose.getX();
      double gravityXSum = 0.0;
      int contactCount = 0;

      for (int moduleIndex = 0; moduleIndex < moduleOffsets.length; moduleIndex++) {
        double worldX = currentRobotX + rotatedModuleOffsets[moduleIndex].getX();
        moduleZVelocities[moduleIndex] += FuelSim.GRAVITY.getZ() * dt;
        moduleZPositions[moduleIndex] += moduleZVelocities[moduleIndex] * dt;

        for (int lineIndex = 0; lineIndex < BUMP_LINE_STARTS.length; lineIndex++) {
          double gravityX =
              handleModuleBumpCollision(
                  moduleIndex,
                  worldX,
                  worldY[moduleIndex],
                  onRamp ? simXVelocity : mapleVx,
                  lineIndex);
          if (!Double.isNaN(gravityX)) {
            gravityXSum += gravityX;
            contactCount++;
          }
        }

        if (moduleZPositions[moduleIndex] < 0.0) {
          moduleZPositions[moduleIndex] = 0.0;
          if (moduleZVelocities[moduleIndex] < 0.0) {
            moduleZVelocities[moduleIndex] = -moduleZVelocities[moduleIndex] * BUMP_COR;
          }
        }
      }

      if (contactCount > 0) {
        if (!onRamp) {
          onRamp = true;
          simXPosition = robotPose.getX();
          simXVelocity = mapleVx;
        }
        simXVelocity += (gravityXSum / contactCount) * contactFactor * dt;
        simXVelocity *= Math.max(0.0, 1.0 - FuelSim.FRICTION * dt);
        simXPosition += simXVelocity * dt;
      } else if (onRamp) {
        if (allModulesFlat()) {
          onRamp = false;
        } else {
          simXPosition += simXVelocity * dt;
        }
      }
    }

    return computePose3d(robotPose);
  }

  private double handleModuleBumpCollision(
      int moduleIndex, double worldX, double worldY, double currentXVelocity, int lineIndex) {
    Translation3d lineStart = BUMP_LINE_STARTS[lineIndex];
    Translation3d lineEnd = BUMP_LINE_ENDS[lineIndex];
    if (worldY < lineStart.getY() || worldY > lineEnd.getY()) {
      return Double.NaN;
    }

    Translation2d start = new Translation2d(lineStart.getX(), lineStart.getZ());
    Translation2d end = new Translation2d(lineEnd.getX(), lineEnd.getZ());
    Translation2d modulePosition = new Translation2d(worldX, moduleZPositions[moduleIndex]);
    Translation2d lineVector = end.minus(start);
    Translation2d toModule = modulePosition.minus(start);
    double projectionT = toModule.dot(lineVector) / lineVector.getSquaredNorm();
    Translation2d projected = start.plus(lineVector.times(projectionT));

    if (projected.getDistance(start) + projected.getDistance(end)
        > lineVector.getNorm() + SEGMENT_PROJECTION_TOLERANCE) {
      return Double.NaN;
    }

    double distance = modulePosition.getDistance(projected);
    if (distance > WHEEL_RADIUS_METERS) {
      return Double.NaN;
    }

    double normalX = -lineVector.getY() / lineVector.getNorm();
    double normalZ = lineVector.getX() / lineVector.getNorm();
    moduleZPositions[moduleIndex] += normalZ * (WHEEL_RADIUS_METERS - distance);

    double velocityDotNormal =
        currentXVelocity * normalX + moduleZVelocities[moduleIndex] * normalZ;
    if (velocityDotNormal < 0.0) {
      moduleZVelocities[moduleIndex] += normalZ * (-(1.0 + BUMP_COR) * velocityDotNormal);
    }

    return -FuelSim.GRAVITY.getZ() * normalX * normalZ;
  }

  private boolean allModulesFlat() {
    for (double moduleZPosition : moduleZPositions) {
      if (moduleZPosition > 0.01) {
        return false;
      }
    }
    return true;
  }

  private Pose3d computePose3d(Pose2d robotPose) {
    double frontZ = (moduleZPositions[0] + moduleZPositions[1]) / 2.0;
    double backZ = (moduleZPositions[2] + moduleZPositions[3]) / 2.0;
    double leftZ = (moduleZPositions[0] + moduleZPositions[2]) / 2.0;
    double rightZ = (moduleZPositions[1] + moduleZPositions[3]) / 2.0;
    double centerZ = (frontZ + backZ) / 2.0 + CHASSIS_HEIGHT_METERS;
    double pitch = -Math.atan2(frontZ - backZ, frontBackDistance);
    double roll = Math.atan2(leftZ - rightZ, leftRightDistance);
    double visualX = onRamp ? simXPosition : robotPose.getX();

    return new Pose3d(
        visualX,
        robotPose.getY(),
        centerZ,
        new Rotation3d(roll, pitch, robotPose.getRotation().getRadians()));
  }
}
