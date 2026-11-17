package frc.robot.subsystems.turret.demo;

import static frc.robot.Constants.DemoConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;
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
  private static final int FIDUCIAL_FIELDS = 7;

  private final Drive drive;
  private final Camera[] cameras;

  private Translation3d tagRobotRelative = null;
  private Rotation2d lastGyro = null;
  private double lastUpdateTimestamp = Double.NaN;
  private double lastSeenTimestamp = Double.NEGATIVE_INFINITY;
  private int trackedTagId = DEMO_TAG_ID;

  private static final class Camera {
    final String name;
    final String statusKey;
    final Transform3d robotToCamera;
    final DoubleArraySubscriber rawFiducials;
    long lastAcceptedTimestamp;

    Camera(String name, Transform3d robotToCamera) {
      this.name = name;
      statusKey = "Demo/" + name + "/Status";
      this.robotToCamera = robotToCamera;
      rawFiducials =
          NetworkTableInstance.getDefault()
              .getTable(name)
              .getDoubleArrayTopic("rawfiducials")
              .subscribe(
                  new double[0], PubSubOption.periodic(0.02), PubSubOption.keepDuplicates(true));
    }
  }

  DemoTargetTracker(Drive drive) {
    this.drive = drive;
    cameras =
        new Camera[] {
          new Camera(VisionConstants.camera0Name, VisionConstants.robotToCamera0),
          new Camera(VisionConstants.camera1Name, VisionConstants.robotToCamera1)
        };
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
    for (Camera camera : cameras) {
      camera.lastAcceptedTimestamp = 0;
    }
  }

  int getTagId() {
    double id = SmartDashboard.getNumber(TAG_ID_KEY, DEMO_TAG_ID);
    return id >= 0 && id <= Integer.MAX_VALUE && id == Math.rint(id) ? (int) id : -1;
  }

  /** Aim point relative to robot center (x fwd, y left, z above floor), empty if never seen. */
  Optional<Translation3d> getAimPointRobotRelative() {
    if (tagRobotRelative == null) {
      return Optional.empty();
    }
    double above = SmartDashboard.getNumber(HEIGHT_ABOVE_TAG_KEY, DEFAULT_HEIGHT_ABOVE_TAG_METERS);
    return Optional.of(tagRobotRelative.plus(new Translation3d(0.0, 0.0, above)));
  }

  /** Call once per loop. */
  void update() {
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
    if (tagId != trackedTagId) {
      clearTarget();
      trackedTagId = tagId;
    }
    for (Camera camera : cameras) {
      if (tagId < 0) {
        SmartDashboard.putString(camera.statusKey, "Set Demo/Tag ID");
        continue;
      }
      var frame = camera.rawFiducials.getAtomic();
      double sampleAgeSeconds = (NetworkTablesJNI.now() - frame.timestamp) * 1.0e-6;
      if (frame.timestamp == 0 || sampleAgeSeconds > MAX_LIMELIGHT_SAMPLE_AGE_SECS) {
        SmartDashboard.putString(camera.statusKey, "No fresh rawfiducials data");
        continue;
      }
      int index = findTag(frame.value, tagId);
      if (index < 0) {
        SmartDashboard.putString(camera.statusKey, "Configured tag absent from camera results");
        continue;
      }
      var measurement =
          toRobotFrame(
              frame.value[index + 1],
              frame.value[index + 2],
              frame.value[index + 4],
              camera.robotToCamera);
      if (measurement.isEmpty()) {
        SmartDashboard.putString(
            camera.statusKey, "Tag detected without usable angles or distance");
        continue;
      }
      Translation3d robotRelative = measurement.get();
      Logger.recordOutput("Demo/" + camera.name + "/RawTagRobotRelative", robotRelative);
      SmartDashboard.putNumber("Demo/Raw Tag Robot X", robotRelative.getX());
      SmartDashboard.putNumber("Demo/Raw Tag Robot Y", robotRelative.getY());
      SmartDashboard.putNumber("Demo/Raw Tag Robot Z", robotRelative.getZ());
      if (!isPlausible(robotRelative)) {
        SmartDashboard.putString(camera.statusKey, "Tag outside demo height or range limits");
        continue;
      }
      SmartDashboard.putString(camera.statusKey, "Tracking");
      if (frame.timestamp != camera.lastAcceptedTimestamp) {
        acceptMeasurement(robotRelative);
        camera.lastAcceptedTimestamp = frame.timestamp;
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

  /** Finds the configured ID in a rawfiducials frame, including non-primary tags. */
  static int findTag(double[] fiducials, int tagId) {
    if (fiducials.length % FIDUCIAL_FIELDS != 0) {
      return -1;
    }
    for (int index = 0; index < fiducials.length; index += FIDUCIAL_FIELDS) {
      if (fiducials[index] == tagId) {
        return index;
      }
    }
    return -1;
  }

  /** Reconstructs the camera-to-tag translation and applies the camera mount pose. */
  static Optional<Translation3d> toRobotFrame(
      double txDegrees, double tyDegrees, double distanceMeters, Transform3d robotToCamera) {
    if (!(Math.abs(txDegrees) < 90.0 && Math.abs(tyDegrees) < 90.0)
        || !Double.isFinite(distanceMeters)
        || distanceMeters <= 0.0) {
      return Optional.empty();
    }
    // Limelight angles are positive right/up. WPILib camera axes are forward/left/up.
    Translation3d ray =
        new Translation3d(
            1.0, -Math.tan(Math.toRadians(txDegrees)), Math.tan(Math.toRadians(tyDegrees)));
    Translation3d cameraToTag = ray.times(distanceMeters / ray.getNorm());
    return Optional.of(
        robotToCamera.getTranslation().plus(cameraToTag.rotateBy(robotToCamera.getRotation())));
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
