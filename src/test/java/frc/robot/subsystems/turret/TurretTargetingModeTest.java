package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class TurretTargetingModeTest {
  @Test
  void trenchOverridesNeutralZoneFerry() {
    assertEquals(
        TurretTargeting.TargetingMode.HUB_AUTO, TurretTargeting.selectTargetingMode(true, true));
  }

  @Test
  void neutralZoneEnablesAutoFerryWhenTrenchNotActive() {
    assertEquals(
        TurretTargeting.TargetingMode.FERRY_AUTO, TurretTargeting.selectTargetingMode(false, true));
  }

  @Test
  void defaultIsHubWhenNotInNeutralZone() {
    assertEquals(
        TurretTargeting.TargetingMode.HUB_AUTO, TurretTargeting.selectTargetingMode(false, false));
  }
}
