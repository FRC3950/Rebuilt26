package frc.robot.subsystems.turret.demo;

import static frc.robot.Constants.DemoConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DemoConstants.LimelightRobotSpaceAxes;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks the demo AprilTag in the ROBOT frame (x forward, y left, z up). Field odometry is not
 * used. When a Limelight sees the tag, the stored position is pulled toward the measurement. When
 * the tag is out of view, the stored position is moved by the robot's own motion (wheel speeds and
 * raw gyro), so the turrets keep aiming at it.
 *
 * <p>In simulation the tag comes from the "Demo/Sim Tag *" dashboard entries (sim field frame).
 */
class DemoTargetTracker {
  private static final String TAG_ID_KEY = "Demo/Tag ID";
  private static final String HEIGHT_ABOVE_TAG_KEY = "Demo/Height Above Tag M";
  private static final String SIM_TAG_X_KEY = "Demo/Sim Tag X";
  private static final String SIM_TAG_Y_KEY = "Demo/Sim Tag Y";
  private static final String SIM_TAG_Z_KEY = "Demo/Sim Tag Z";

  private final Drive drive;
  private final Camera[] cameras;

  private Translation3d tagRobotRelative = null;
  private Rotation2d lastGyro = null;
  private double lastUpdateTimestamp = Double.NaN;
  private double lastSeenTimestamp = Double.NEGATIVE_INFINITY;
  private int publishedPriorityId = Integer.MIN_VALUE;

  private static final class Camera {
    final String name;
    final DoubleSubscriber tv;
    final DoubleSubscriber tid;
    final DoubleArraySubscriber targetPoseRobotSpace;
    final DoublePublisher priorityId;

    Camera(String name) {
      NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
      this.name = name;
      tv = table.getDoubleTopic("tv").subscribe(0.0);
      tid = table.getDoubleTopic("tid").subscribe(-1.0);
      targetPoseRobotSpace =
          table.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[] {});
      priorityId = table.getDoubleTopic("priorityid").publish();
    }
  }

  DemoTargetTracker(Drive drive, String... limelightNames) {
    this.drive = drive;
    cameras = new Camera[limelightNames.length];
    for (int i = 0; i < limelightNames.length; i++) {
      cameras[i] = new Camera(limelightNames[i]);
    }
    SmartDashboard.setDefaultNumber(TAG_ID_KEY, DEMO_TAG_ID);
    SmartDashboard.setDefaultNumber(HEIGHT_ABOVE_TAG_KEY, DEFAULT_HEIGHT_ABOVE_TAG_METERS);
    if (Constants.currentMode == Constants.Mode.SIM) {
      SmartDashboard.setDefaultNumber(SIM_TAG_X_KEY, 8.0);
      SmartDashboard.setDefaultNumber(SIM_TAG_Y_KEY, 4.0);
      SmartDashboard.setDefaultNumber(SIM_TAG_Z_KEY, 1.5);
    }
  }

  void clearTarget() {
    tagRobotRelative = null;
    lastSeenTimestamp = Double.NEGATIVE_INFINITY;
  }

  int getTagId() {
    return (int) Math.round(SmartDashboard.getNumber(TAG_ID_KEY, DEMO_TAG_ID));
  }

  /** Aim point relative to robot center (x fwd, y left, z above floor), empty if never seen. */
  Optional<Translation3d> getAimPointRobotRelative() {
    if (tagRobotRelative == null) {
      return Optional.empty();
    }
    double above = SmartDashboard.getNumber(HEIGHT_ABOVE_TAG_KEY, DEFAULT_HEIGHT_ABOVE_TAG_METERS);
    return Optional.of(tagRobotRelative.plus(new Translation3d(0.0, 0.0, above)));
  }

  /** Call once per loop. {@code active} = demo mode selected (controls Limelight priorityid). */
  void update(boolean active) {
    publishPriorityId(active);
    propagateWithRobotMotion();
    if (Constants.currentMode == Constants.Mode.SIM) {
      acceptMeasurement(simMeasurement());
    } else {
      readLimelights();
    }
    publishTelemetry();
  }

  /** Moves the stored target opposite to how the robot moved since last loop. */
  private void propagateWithRobotMotion() {
    double now = Timer.getFPGATimestamp();
    Rotation2d gyro = drive.getRawGyroRotation();
    if (tagRobotRelative != null && lastGyro != null && Double.isFinite(lastUpdateTimestamp)) {
      double dt = now - lastUpdateTimestamp;
      tagRobotRelative =
          propagate(tagRobotRelative, drive.getRobotRelativeSpeeds(), gyro.minus(lastGyro), dt);
    }
    lastGyro = gyro;
    lastUpdateTimestamp = now;
  }

  static Translation3d propagate(
      Translation3d target, ChassisSpeeds robotSpeeds, Rotation2d robotRotationDelta, double dt) {
    Translation2d moved =
        target
            .toTranslation2d()
            .minus(
                new Translation2d(
                    robotSpeeds.vxMetersPerSecond * dt, robotSpeeds.vyMetersPerSecond * dt))
            .rotateBy(robotRotationDelta.unaryMinus());
    return new Translation3d(moved.getX(), moved.getY(), target.getZ());
  }

  private void readLimelights() {
    int tagId = getTagId();
    if (tagId < 0) {
      return;
    }
    for (Camera camera : cameras) {
      if (camera.tv.get() < 0.5 || (int) Math.round(camera.tid.get()) != tagId) {
        continue;
      }
      double[] raw = camera.targetPoseRobotSpace.get();
      if (raw.length < 6) {
        continue;
      }
      Translation3d robotRelative = toRobotFrame(raw, LIMELIGHT_ROBOT_SPACE_AXES);
      Logger.recordOutput("Demo/" + camera.name + "/RawTagRobotRelative", robotRelative);
      SmartDashboard.putNumber("Demo/Raw Tag Robot X", robotRelative.getX());
      SmartDashboard.putNumber("Demo/Raw Tag Robot Y", robotRelative.getY());
      SmartDashboard.putNumber("Demo/Raw Tag Robot Z", robotRelative.getZ());
      if (isPlausible(robotRelative)) {
        acceptMeasurement(robotRelative);
      }
    }
  }

  /** Sim stand-in for a Limelight: dashboard tag in the sim field frame, seen from sim pose. */
  private Translation3d simMeasurement() {
    Pose2d pose = drive.getPose();
    Translation2d tagField =
        new Translation2d(
            SmartDashboard.getNumber(SIM_TAG_X_KEY, 8.0),
            SmartDashboard.getNumber(SIM_TAG_Y_KEY, 4.0));
    Translation2d rel =
        tagField.minus(pose.getTranslation()).rotateBy(pose.getRotation().unaryMinus());
    return new Translation3d(rel.getX(), rel.getY(), SmartDashboard.getNumber(SIM_TAG_Z_KEY, 1.5));
  }

  private void acceptMeasurement(Translation3d measurement) {
    if (tagRobotRelative == null
        || tagRobotRelative.getDistance(measurement) > TARGET_RESET_JUMP_METERS) {
      tagRobotRelative = measurement;
    } else {
      tagRobotRelative = tagRobotRelative.interpolate(measurement, TARGET_FILTER_ALPHA);
    }
    lastSeenTimestamp = Timer.getFPGATimestamp();
  }

  /**
   * Converts a Limelight targetpose_robotspace array to WPILib robot axes (x fwd, y left, z up).
   */
  static Translation3d toRobotFrame(double[] raw, LimelightRobotSpaceAxes axes) {
    return switch (axes) {
      case FORWARD_LEFT_UP -> new Translation3d(raw[0], raw[1], raw[2]);
      case RIGHT_DOWN_FORWARD -> new Translation3d(raw[2], -raw[0], -raw[1]);
    };
  }

  static boolean isPlausible(Translation3d robotRelative) {
    double z = robotRelative.getZ();
    double range = robotRelative.toTranslation2d().getNorm();
    return Double.isFinite(z)
        && z >= MIN_TAG_HEIGHT_METERS
        && z <= MAX_TAG_HEIGHT_METERS
        && range > 0.1
        && range <= MAX_TAG_RANGE_METERS;
  }

  private void publishPriorityId(boolean active) {
    // Only this class writes priorityid. -1 releases it back to normal targeting.
    int desired = active ? getTagId() : -1;
    if (desired == publishedPriorityId) {
      return;
    }
    for (Camera camera : cameras) {
      camera.priorityId.set(desired);
    }
    publishedPriorityId = desired;
  }

  private void publishTelemetry() {
    boolean hasTarget = tagRobotRelative != null;
    SmartDashboard.putBoolean("Demo/Has Target", hasTarget);
    SmartDashboard.putNumber(
        "Demo/Seconds Since Seen", hasTarget ? Timer.getFPGATimestamp() - lastSeenTimestamp : -1.0);
    Logger.recordOutput("Demo/HasTarget", hasTarget);
    getAimPointRobotRelative()
        .ifPresent(
            aim -> {
              SmartDashboard.putNumber("Demo/Aim Robot X", aim.getX());
              SmartDashboard.putNumber("Demo/Aim Robot Y", aim.getY());
              SmartDashboard.putNumber("Demo/Aim Z", aim.getZ());
              Logger.recordOutput("Demo/AimPointRobotRelative", aim);
            });
  }
}
