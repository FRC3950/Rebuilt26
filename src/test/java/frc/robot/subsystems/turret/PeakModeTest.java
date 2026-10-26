package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Distancer;
import java.util.List;
import org.junit.jupiter.api.Test;

class PeakModeTest {
  @Test
  void switchSelectsTableAndStartsOff() {
    assertTrue(HAL.initialize(500, 0));
    try (NetworkTableInstance instance = NetworkTableInstance.create()) {
      instance.startLocal();
      SmartDashboard.setNetworkTableInstance(instance);
      try {
        SmartDashboard.putBoolean("Peak Mode", true);
        GetAdjustedShot.initializeDashboard();
        assertFalse(GetAdjustedShot.isPeakModeActive());
        Distancer standard = GetAdjustedShot.getTableShot(3.0);

        SmartDashboard.putBoolean("Peak Mode", true);
        assertTrue(GetAdjustedShot.isPeakModeActive());
        Distancer peak = GetAdjustedShot.getTableShot(3.0);
        assertEquals(rowAt("shot_table_peak.json", 3.0).rps, peak.flywheelRps(), 1e-9);
        assertEquals(rowAt("shot_table.json", 3.0).rps, standard.flywheelRps(), 1e-9);

        SmartDashboard.putBoolean("Peak Mode", false);
        assertEquals(standard, GetAdjustedShot.getTableShot(3.0));
      } finally {
        SmartDashboard.setNetworkTableInstance(NetworkTableInstance.getDefault());
      }
    }
  }

  @Test
  void hoodAndFlywheelNeverDecreaseWithDistance() {
    for (String file : List.of("shot_table.json", "shot_table_peak.json")) {
      List<Distancer.Row> rows = Distancer.loadRowsFromDeploy(file);
      assertFalse(rows.isEmpty(), file);
      for (int i = 1; i < rows.size(); i++) {
        assertTrue(
            rows.get(i).hoodDeg >= rows.get(i - 1).hoodDeg, file + " hood at " + rows.get(i).d);
        assertTrue(rows.get(i).rps >= rows.get(i - 1).rps, file + " rps at " + rows.get(i).d);
      }
    }
  }

  private static Distancer.Row rowAt(String file, double d) {
    return Distancer.loadRowsFromDeploy(file).stream()
        .filter(r -> Math.abs(r.d - d) < 1e-9)
        .findFirst()
        .orElseThrow();
  }
}
