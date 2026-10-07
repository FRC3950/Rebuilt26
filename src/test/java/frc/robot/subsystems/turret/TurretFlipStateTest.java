package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class TurretFlipStateTest {
  @Test
  void wrapFlipStaysActiveThroughTheLastHalfOfTheRotation() {
    double target = Turret.selectSafeSetpointDegrees(10.0, 0.0, -380.0, 1.0);
    boolean flipping = Turret.nextFlippingState(false, 0.0, target, 0.0);
    assertTrue(flipping);
    flipping = Turret.nextFlippingState(flipping, target, target, -200.0);
    assertTrue(flipping);
    assertFalse(Turret.nextFlippingState(flipping, target, target, -346.0));
  }

  @Test
  void normalTrackingAcrossZeroDoesNotCountAsAFlip() {
    double target = Turret.selectSafeSetpointDegrees(1.0, -359.0, -380.0, 1.0);
    assertFalse(Turret.nextFlippingState(false, -359.0, target, -360.0));
  }

  @Test
  void interruptedFlipIsDetectedAgainFromMeasuredPosition() {
    assertTrue(Turret.nextFlippingState(false, -350.0, -350.0, -100.0));
  }

  @Test
  void invalidFeedbackCannotFinishAnActiveFlip() {
    assertTrue(Turret.nextFlippingState(true, -350.0, -350.0, Double.NaN));
    assertTrue(Turret.nextFlippingState(true, -350.0, -350.0, Double.POSITIVE_INFINITY));
  }
}
