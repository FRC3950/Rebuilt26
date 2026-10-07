package frc.robot.controls;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.junit.jupiter.api.Test;

class TuneDashboardTest {
  @Test
  void publishesOnlyWhileActiveAndRestoresInputsOnReentry() {
    TuneModeBindings.setDashboardActive(false, 13.0);
    assertFalse(SmartDashboard.containsKey("Tune/Hood Deg"));
    assertFalse(SmartDashboard.containsKey("Tune/Flywheel RPS"));

    TuneModeBindings.setDashboardActive(true, 13.0);
    assertTrue(SmartDashboard.containsKey("Tune/Hood Deg"));
    assertTrue(SmartDashboard.containsKey("Tune/Save Status"));
    SmartDashboard.putNumber("Tune/Hood Deg", 20.0);
    SmartDashboard.putNumber("Tune/Flywheel RPS", 35.0);
    SmartDashboard.putNumber("Tune/Left Turret Measured RPS", 34.5);
    SmartDashboard.putNumber("Tune/Table Hood Deg", 19.0);

    TuneModeBindings.setDashboardActive(false, 13.0);
    assertFalse(SmartDashboard.containsKey("Tune/Hood Deg"));
    assertFalse(SmartDashboard.containsKey("Tune/Flywheel RPS"));
    assertFalse(SmartDashboard.containsKey("Tune/Save Status"));
    assertFalse(SmartDashboard.containsKey("Tune/Left Turret Measured RPS"));
    assertFalse(SmartDashboard.containsKey("Tune/Table Hood Deg"));

    TuneModeBindings.setDashboardActive(true, 13.0);
    assertEquals(20.0, SmartDashboard.getNumber("Tune/Hood Deg", -1.0));
    assertEquals(35.0, SmartDashboard.getNumber("Tune/Flywheel RPS", -1.0));
    TuneModeBindings.setDashboardActive(false, 13.0);
  }
}
