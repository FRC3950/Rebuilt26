package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class TurretWrapSelectionTest {
  @Test
  void firstTargetNearZeroUsesMeasuredNegativeTurnForEitherTurret() {
    assertEquals(-360.0, Turret.selectSafeSetpointDegrees(0.0, -355.0, -380.0, 1.0));
    assertEquals(-360.0, Turret.selectSafeSetpointDegrees(0.0, -355.0, -365.0, 10.0));
    assertEquals(0.0, Turret.selectSafeSetpointDegrees(0.0, -5.0, -380.0, 1.0));
    assertEquals(0.0, Turret.selectSafeSetpointDegrees(0.0, -5.0, -365.0, 10.0));
  }

  @Test
  void subsequentTargetsKeepTheSelectedWrapAcrossZero() {
    double firstTarget = Turret.selectSafeSetpointDegrees(0.0, -355.0, -380.0, 1.0);
    assertEquals(-359.0, Turret.selectSafeSetpointDegrees(1.0, firstTarget, -380.0, 1.0));
    assertEquals(-361.0, Turret.selectSafeSetpointDegrees(-1.0, firstTarget, -380.0, 1.0));
  }

  @Test
  void onlyLegalCandidateWinsEvenWhenReferenceIsNearTheOtherWrap() {
    assertEquals(-350.0, Turret.selectSafeSetpointDegrees(10.0, 0.0, -380.0, 1.0));
    assertEquals(-10.0, Turret.selectSafeSetpointDegrees(-10.0, -360.0, -365.0, 10.0));
  }
}
