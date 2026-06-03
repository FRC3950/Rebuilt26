package frc.robot.shotmap;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class HubShotPhysicsTest {
  @Test
  void idealProjectileScoresKnownSimpleShotWithoutDrag() {
    HubShotPhysics physics =
        new HubShotPhysics(
            new HubShotPhysics.Parameters(
                0.0, 1.0, 0.25, 0.0, 2.0, 0.3, 1.0, false, 0.0, 0.0, 0.0, 0.0));

    HubShotPhysics.Result result =
        physics.evaluate(1.0, 0.0, 45.0, Math.sqrt(2.0) / (2.0 * Math.PI * 0.3));

    assertTrue(result.scored());
    assertEquals(1.0, result.tofSec(), 1e-3);
    assertEquals(0.25, result.marginMeters(), 1e-3);
  }
}
