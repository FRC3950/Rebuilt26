package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.junit.jupiter.api.Test;

class TuneDashboardTest {
  @Test
  void publishesControlsPreservesInputsAndResetsRunSwitchAtStartup() {
    assertTrue(HAL.initialize(500, 0));
    try (NetworkTableInstance instance = NetworkTableInstance.create()) {
      instance.startLocal();
      SmartDashboard.setNetworkTableInstance(instance);
      try {
        Tune.initializeDashboard(13.0);
        assertTrue(SmartDashboard.containsKey("Tune/Hood Deg"));
        assertFalse(SmartDashboard.containsKey("Tune/Save Status"));
        assertFalse(SmartDashboard.containsKey("Tune/Save Point/.type"));
        assertFalse(Tune.useTuneValues());

        SmartDashboard.putNumber("Tune/Hood Deg", 20.0);
        SmartDashboard.putNumber("Tune/Flywheel RPS", 35.0);
        SmartDashboard.putBoolean("Tune/Use Tune Values", true);
        assertTrue(Tune.useTuneValues());
        SmartDashboard.putBoolean("Tune/Use Tune Values", false);
        assertFalse(Tune.useTuneValues());
        assertEquals(20.0, SmartDashboard.getNumber("Tune/Hood Deg", -1.0));
        assertEquals(35.0, SmartDashboard.getNumber("Tune/Flywheel RPS", -1.0));

        SmartDashboard.putBoolean("Tune/Use Tune Values", true);
        Tune.initializeDashboard(13.0);
        assertFalse(Tune.useTuneValues());
        assertEquals(20.0, SmartDashboard.getNumber("Tune/Hood Deg", -1.0));
        assertEquals(35.0, SmartDashboard.getNumber("Tune/Flywheel RPS", -1.0));
      } finally {
        SmartDashboard.setNetworkTableInstance(NetworkTableInstance.getDefault());
      }
    }
  }

  @Test
  void rejectsUnsafeInputsAndAllowsZeroSpeed() {
    assertFalse(Tune.validateTuneSetpoint(Double.NaN, 20.0).valid());
    assertFalse(Tune.validateTuneSetpoint(-100.0, 20.0).valid());
    assertFalse(Tune.validateTuneSetpoint(13.0, Double.POSITIVE_INFINITY).valid());
    assertFalse(Tune.validateTuneSetpoint(13.0, -1.0).valid());
    assertTrue(Tune.validateTuneSetpoint(13.0, 0.0).valid());
  }
}
