package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.HandheldTagObservation;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.RawFiducialObservation;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class DemoContainerHandheldTargetTest {
  private static final double EPSILON = 1e-9;

  @Test
  void selectedTagIdFiltersRawObservations() {
    TestVisionIO io =
        new TestVisionIO(
            new RawFiducialObservation(2, 0.0, 0.0, 1.0, 3.0, 3.0, 0.0, 0.0),
            new RawFiducialObservation(5, 10.0, 0.0, 3.0, 2.0, 2.0, 0.0, 0.0));
    Vision vision = new Vision((pose, timestamp, stdDevs) -> {}, io);
    io.refreshTimestamps();

    vision.periodic();

    Optional<HandheldTagObservation> selected = vision.getBestHandheldTagObservation(2);
    assertTrue(selected.isPresent());
    assertEquals(2, selected.get().tagId());
    assertEquals(0.0, selected.get().tx().getDegrees(), EPSILON);
  }

  @Test
  void invalidHandheldObservationsAreRejected() {
    double now = 10.0;

    assertFalse(
        Vision.isUsableHandheldTagObservation(
            new HandheldTagObservation(
                0, 2, Rotation2d.kZero, 1.0, 2.0, 0.0, now - 1.0, Rotation2d.kZero),
            Vision.ANY_HANDHELD_TAG_ID,
            now));
    assertFalse(
        Vision.isUsableHandheldTagObservation(
            new HandheldTagObservation(
                0, 2, Rotation2d.kZero, 1.0, 2.0, 1.0, now, Rotation2d.kZero),
            Vision.ANY_HANDHELD_TAG_ID,
            now));
    assertFalse(
        Vision.isUsableHandheldTagObservation(
            new HandheldTagObservation(
                0, 2, Rotation2d.kZero, 0.0, 2.0, 0.0, now, Rotation2d.kZero),
            Vision.ANY_HANDHELD_TAG_ID,
            now));
    assertFalse(
        Vision.isUsableHandheldTagObservation(
            new HandheldTagObservation(
                0, 2, Rotation2d.kZero, 1.0, 0.05, 0.0, now, Rotation2d.kZero),
            Vision.ANY_HANDHELD_TAG_ID,
            now));
  }

  @Test
  void targetTranslationChangesWithRobotPose() {
    HandheldTagObservation observation =
        new HandheldTagObservation(0, 2, Rotation2d.kZero, 1.0, 3.0, 0.0, 0.0, Rotation2d.kZero);

    Translation2d firstTarget =
        DemoContainer.calculateTargetTranslation(
            new Pose2d(1.0, 2.0, Rotation2d.kZero), observation);
    Translation2d secondTarget =
        DemoContainer.calculateTargetTranslation(
            new Pose2d(1.0, 2.0, Rotation2d.kCCW_90deg), observation);

    assertEquals(4.0, firstTarget.getX(), EPSILON);
    assertEquals(2.0, firstTarget.getY(), EPSILON);
    assertEquals(1.0, secondTarget.getX(), EPSILON);
    assertEquals(5.0, secondTarget.getY(), EPSILON);
  }

  @Test
  void tagForwardHeadingPointsFromRobotToTarget() {
    Rotation2d direction =
        DemoContainer.calculateTagForwardDirection(
            new Pose2d(1.0, 1.0, Rotation2d.kZero), new Translation2d(1.0, 4.0));

    assertEquals(90.0, direction.getDegrees(), EPSILON);
  }

  @Test
  void poseFusionGateBlocksVisionConsumer() {
    AtomicInteger consumerCalls = new AtomicInteger();
    TestVisionIO io =
        new TestVisionIO(
            new PoseObservation(
                0.0,
                new Pose3d(1.0, 1.0, 0.0, Rotation3d.kZero),
                0.0,
                1,
                2.0,
                PoseObservationType.MEGATAG_1));
    Vision vision = new Vision((pose, timestamp, stdDevs) -> consumerCalls.incrementAndGet(), io);
    vision.setPoseFusionAllowedSupplier(() -> false);

    vision.periodic();

    assertEquals(0, consumerCalls.get());
  }

  private static class TestVisionIO implements VisionIO {
    private final RawFiducialObservation[] rawFiducials;
    private final PoseObservation[] poseObservations;

    TestVisionIO(RawFiducialObservation... rawFiducials) {
      this.rawFiducials = rawFiducials;
      this.poseObservations = new PoseObservation[0];
    }

    TestVisionIO(PoseObservation... poseObservations) {
      this.rawFiducials = new RawFiducialObservation[0];
      this.poseObservations = poseObservations;
    }

    void refreshTimestamps() {
      for (int i = 0; i < rawFiducials.length; i++) {
        RawFiducialObservation observation = rawFiducials[i];
        rawFiducials[i] =
            new RawFiducialObservation(
                observation.tagId(),
                observation.txncDeg(),
                observation.tyncDeg(),
                observation.targetArea(),
                observation.distanceToCamera(),
                observation.distanceToRobot(),
                observation.ambiguity(),
                edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
      }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      inputs.connected = true;
      inputs.poseObservations = poseObservations;
      inputs.tagIds = new int[] {2};
    }

    @Override
    public RawFiducialObservation[] getRawFiducialObservations() {
      return rawFiducials;
    }
  }
}
