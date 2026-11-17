package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class VisionLocalizationTest {
  @Test
  void localizationCanBeDisabledWhileCamerasStillReportPoses() {
    assertTrue(HAL.initialize(500, 0));
    var localizationEnabled = new AtomicBoolean(false);
    var fusedCount = new AtomicInteger();
    var camera =
        new VisionIO() {
          @Override
          public void updateInputs(VisionIOInputs inputs) {
            inputs.connected = true;
            inputs.poseObservations =
                new PoseObservation[] {
                  new PoseObservation(
                      1.0,
                      new Pose3d(2, 2, 0, Rotation3d.kZero),
                      0.0,
                      1,
                      2,
                      PoseObservationType.MEGATAG_1)
                };
          }
        };
    var vision =
        new Vision(
            (pose, time, noise) -> fusedCount.incrementAndGet(), localizationEnabled::get, camera);
    try {
      vision.periodic();
      assertEquals(0, fusedCount.get());
      localizationEnabled.set(true);
      vision.periodic();
      assertEquals(1, fusedCount.get());
      localizationEnabled.set(false);
      vision.periodic();
      assertEquals(1, fusedCount.get());
    } finally {
      CommandScheduler.getInstance().unregisterSubsystem(vision);
    }
  }
}
