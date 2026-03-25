package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.turret.TurretTargeting.OutputMode;
import org.junit.jupiter.api.Test;

class TurretTargetingTest {
  @Test
  void manualSetpointsOnlyRunWhenEnabledInManualMode() {
    assertTrue(
        TurretTargeting.shouldRunManualSetpoints(OutputMode.MANUAL_SETPOINTS_WHEN_ENABLED, true));
    assertFalse(
        TurretTargeting.shouldRunManualSetpoints(OutputMode.MANUAL_SETPOINTS_WHEN_ENABLED, false));
    assertFalse(TurretTargeting.shouldRunManualSetpoints(OutputMode.AZIMUTH_ONLY, true));
  }
}
