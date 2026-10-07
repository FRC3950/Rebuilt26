package frc.robot.subsystems.turret.turret_base;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class StartupWindowTest {
  private double select(double absolute) {
    return StartupWindow.selectSensorRotations(absolute, -180.0, 9.0, 10.0, -380.0, 1.0);
  }

  @Test
  void reconstructsCenterAndBothWindowBoundaries() {
    assertEquals(-5.0, select(0.0), 1e-9);
    assertEquals(-5.25, select(0.75), 1e-9);
    assertEquals(-4.75, select(0.25), 1e-9);
  }

  @Test
  void acceptsEitherAbsoluteDiscontinuityRepresentation() {
    assertEquals(select(0.75), select(-0.25), 1e-9);
    assertEquals(-5.001, select(0.999), 1e-9);
    assertEquals(-4.999, select(0.001), 1e-9);
  }

  @Test
  void rejectsReadingsJustOutsideBothBoundariesAndAtAmbiguousHalfTurn() {
    assertTrue(Double.isNaN(select(0.749)));
    assertTrue(Double.isNaN(select(0.251)));
    assertTrue(Double.isNaN(select(0.5)));
  }

  @Test
  void supportsIndependentCentersAndRightTurretLimits() {
    assertEquals(
        -5.0, StartupWindow.selectSensorRotations(0.0, -180.0, 9.0, 10.0, -365.0, 10.0), 1e-9);
    assertEquals(
        -4.8, StartupWindow.selectSensorRotations(0.2, -172.8, 9.0, 10.0, -365.0, 10.0), 1e-9);
  }

  @Test
  void rejectsInvalidConfigurationAndNonfiniteReadings() {
    assertTrue(Double.isNaN(select(Double.NaN)));
    assertTrue(Double.isNaN(select(Double.POSITIVE_INFINITY)));
    assertTrue(Double.isNaN(StartupWindow.selectSensorRotations(0, -180, 18, 10, -380, 1)));
    assertTrue(Double.isNaN(StartupWindow.selectSensorRotations(0, -180, 9, 0, -380, 1)));
    assertTrue(Double.isNaN(StartupWindow.selectSensorRotations(0, -360, 9, 10, -365, 10)));
    assertTrue(Double.isNaN(StartupWindow.selectSensorRotations(0, Double.NaN, 9, 10, -380, 1)));
  }

  @Test
  void documentsThatAnIdenticalReadingCannotDistinguishAnAdjacentPhysicalSector() {
    // Physical -180 and -144 degrees both produce absolute 0 with a 10:1 motor-side encoder.
    // Selection deliberately depends on the operator's physical-window confirmation.
    double atCenter = (-180.0 / 360.0 * 10.0) % 1.0;
    double oneSensorTurnAway = (-144.0 / 360.0 * 10.0) % 1.0;
    assertEquals(atCenter, oneSensorTurnAway, 1e-9);
    assertEquals(-5.0, select(oneSensorTurnAway), 1e-9);
  }
}
