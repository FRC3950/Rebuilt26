// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.RawFiducialObservation;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  public static final int ANY_HANDHELD_TAG_ID = -1;
  private static final double MAX_HANDHELD_TAG_AGE_SECS = 0.35;
  private static final double MAX_HANDHELD_TAG_AMBIGUITY = 0.90;
  private static final double MIN_HANDHELD_TAG_AREA = 0.0;
  private static final double MIN_HANDHELD_TAG_RANGE_METERS = 0.10;

  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Transform3d[] robotToCameras;
  private final List<Pose3d> allTagPoses = new ArrayList<>(8);
  private final List<Pose3d> allRobotPoses = new ArrayList<>(8);
  private final List<Pose3d> allRobotPosesAccepted = new ArrayList<>(8);
  private final List<Pose3d> allRobotPosesRejected = new ArrayList<>(8);
  private final List<Pose3d> tagPoses = new ArrayList<>(4);
  private final List<Pose3d> robotPoses = new ArrayList<>(4);
  private final List<Pose3d> robotPosesAccepted = new ArrayList<>(4);
  private final List<Pose3d> robotPosesRejected = new ArrayList<>(4);
  private final List<HandheldTagObservation> simHandheldTagObservations = new ArrayList<>(2);
  private BooleanSupplier poseFusionAllowedSupplier = () -> true;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this(
        consumer,
        new Transform3d[] {
          VisionConstants.robotToCamera0, VisionConstants.robotToCamera1,
        },
        io);
  }

  public Vision(VisionConsumer consumer, Transform3d[] robotToCameras, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;
    this.robotToCameras = robotToCameras.clone();

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  public record HandheldTagObservation(
      int cameraIndex,
      int tagId,
      Rotation2d tx,
      double targetArea,
      double distanceToRobotMeters,
      double ambiguity,
      double timestampSecs,
      Rotation2d cameraYaw) {}

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  public void setPoseFusionAllowedSupplier(BooleanSupplier poseFusionAllowedSupplier) {
    this.poseFusionAllowedSupplier = poseFusionAllowedSupplier;
  }

  public boolean isPoseFusionAllowed() {
    return poseFusionAllowedSupplier.getAsBoolean();
  }

  public Optional<HandheldTagObservation> getBestHandheldTagObservation(int selectedTagId) {
    double nowSecs = Timer.getFPGATimestamp();
    HandheldTagObservation bestObservation = null;

    for (int cameraIndex = 0; cameraIndex < inputs.length; cameraIndex++) {
      for (RawFiducialObservation observation : io[cameraIndex].getRawFiducialObservations()) {
        bestObservation =
            chooseBetterHandheldTagObservation(
                bestObservation,
                createHandheldTagObservation(cameraIndex, observation),
                selectedTagId,
                nowSecs);
      }
    }

    for (HandheldTagObservation observation : simHandheldTagObservations) {
      HandheldTagObservation freshObservation =
          new HandheldTagObservation(
              observation.cameraIndex(),
              observation.tagId(),
              observation.tx(),
              observation.targetArea(),
              observation.distanceToRobotMeters(),
              observation.ambiguity(),
              nowSecs,
              observation.cameraYaw());
      bestObservation =
          chooseBetterHandheldTagObservation(
              bestObservation, freshObservation, selectedTagId, nowSecs);
    }

    logHandheldTagObservation(bestObservation);
    return Optional.ofNullable(bestObservation);
  }

  public void setSimHandheldTagObservation(
      int tagId, double txncDeg, double distanceToRobotMeters) {
    simHandheldTagObservations.clear();
    simHandheldTagObservations.add(
        new HandheldTagObservation(
            0,
            tagId,
            Rotation2d.fromDegrees(txncDeg),
            1.0,
            distanceToRobotMeters,
            0.0,
            Timer.getFPGATimestamp(),
            Rotation2d.fromRadians(VisionConstants.robotToCamera0.getRotation().getZ())));
  }

  public void clearSimHandheldTagObservations() {
    simHandheldTagObservations.clear();
  }

  @Override
  public void periodic() {
    boolean shouldFlushNetworkTables = false;
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
      shouldFlushNetworkTables |= inputs[i].hasNewData;
    }

    if (Constants.currentMode == Constants.Mode.REAL && shouldFlushNetworkTables) {
      NetworkTableInstance.getDefault().flush();
    }

    allTagPoses.clear();
    allRobotPoses.clear();
    allRobotPosesAccepted.clear();
    allRobotPosesRejected.clear();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      tagPoses.clear();
      robotPoses.clear();
      robotPosesAccepted.clear();
      robotPosesRejected.clear();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            !isPoseFusionAllowed()
                || observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/PoseFusionAllowed", isPoseFusionAllowed());
  }

  private HandheldTagObservation createHandheldTagObservation(
      int cameraIndex, RawFiducialObservation observation) {
    return new HandheldTagObservation(
        cameraIndex,
        observation.tagId(),
        Rotation2d.fromDegrees(observation.txncDeg()),
        observation.targetArea(),
        observation.distanceToRobot(),
        observation.ambiguity(),
        observation.timestampSecs(),
        getCameraYaw(cameraIndex));
  }

  private Rotation2d getCameraYaw(int cameraIndex) {
    if (cameraIndex < robotToCameras.length) {
      return Rotation2d.fromRadians(robotToCameras[cameraIndex].getRotation().getZ());
    }
    return Rotation2d.kZero;
  }

  private static HandheldTagObservation chooseBetterHandheldTagObservation(
      HandheldTagObservation currentBest,
      HandheldTagObservation candidate,
      int selectedTagId,
      double nowSecs) {
    if (!isUsableHandheldTagObservation(candidate, selectedTagId, nowSecs)) {
      return currentBest;
    }
    if (currentBest == null
        || candidate.targetArea() > currentBest.targetArea()
        || (candidate.targetArea() == currentBest.targetArea()
            && candidate.distanceToRobotMeters() < currentBest.distanceToRobotMeters())) {
      return candidate;
    }
    return currentBest;
  }

  public static boolean isUsableHandheldTagObservation(
      HandheldTagObservation observation, int selectedTagId, double nowSecs) {
    return observation != null
        && (selectedTagId == ANY_HANDHELD_TAG_ID || observation.tagId() == selectedTagId)
        && Double.isFinite(observation.targetArea())
        && observation.targetArea() > MIN_HANDHELD_TAG_AREA
        && Double.isFinite(observation.distanceToRobotMeters())
        && observation.distanceToRobotMeters() > MIN_HANDHELD_TAG_RANGE_METERS
        && Double.isFinite(observation.ambiguity())
        && observation.ambiguity() <= MAX_HANDHELD_TAG_AMBIGUITY
        && Double.isFinite(observation.timestampSecs())
        && nowSecs - observation.timestampSecs() <= MAX_HANDHELD_TAG_AGE_SECS;
  }

  private static void logHandheldTagObservation(HandheldTagObservation observation) {
    boolean valid = observation != null;
    Logger.recordOutput("Vision/HandheldTag/Valid", valid);
    Logger.recordOutput("Vision/HandheldTag/TagId", valid ? observation.tagId() : 0);
    Logger.recordOutput("Vision/HandheldTag/CameraIndex", valid ? observation.cameraIndex() : -1);
    Logger.recordOutput("Vision/HandheldTag/TxDeg", valid ? observation.tx().getDegrees() : 0.0);
    Logger.recordOutput(
        "Vision/HandheldTag/DistanceToRobotMeters",
        valid ? observation.distanceToRobotMeters() : 0.0);
    Logger.recordOutput("Vision/HandheldTag/Ambiguity", valid ? observation.ambiguity() : 0.0);
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
