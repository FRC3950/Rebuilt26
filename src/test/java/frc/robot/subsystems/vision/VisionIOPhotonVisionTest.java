package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

class VisionIOPhotonVisionTest {
  private static final double EPSILON = 1e-9;

  @Test
  void photonTargetCreatesRawFiducialObservationForHandheldTargeting() {
    PhotonTrackedTarget target =
        new PhotonTrackedTarget(
            12.0,
            -3.0,
            4.5,
            0.0,
            14,
            -1,
            -1.0f,
            new Transform3d(2.0, 0.5, 0.25, new Rotation3d()),
            new Transform3d(),
            0.12,
            List.of(new TargetCorner(), new TargetCorner(), new TargetCorner(), new TargetCorner()),
            List.of());
    Transform3d robotToCamera = new Transform3d(0.25, 0.0, 0.5, new Rotation3d());

    VisionIO.RawFiducialObservation observation =
        VisionIOPhotonVision.createRawFiducialObservation(target, robotToCamera, 42.0);

    assertEquals(14, observation.tagId());
    assertEquals(12.0, observation.txncDeg(), EPSILON);
    assertEquals(-3.0, observation.tyncDeg(), EPSILON);
    assertEquals(4.5, observation.targetArea(), EPSILON);
    assertEquals(
        new Translation3d(2.0, 0.5, 0.25).getNorm(), observation.distanceToCamera(), EPSILON);
    assertEquals(
        robotToCamera.plus(target.getBestCameraToTarget()).getTranslation().getNorm(),
        observation.distanceToRobot(),
        EPSILON);
    assertEquals(0.12, observation.ambiguity(), EPSILON);
    assertEquals(42.0, observation.timestampSecs(), EPSILON);
  }
}
