package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.generated.TunerConstants;
import org.junit.jupiter.api.Test;

class PowerOptimizationConstantsTest {
  @Test
  void battlecryMechanismCurrentLimitsAreApplied() {
    var turret = Constants.SubsystemConstants.Turret.flywheelConfig.CurrentLimits;
    assertEquals(65.0, turret.StatorCurrentLimit, 1e-9);
    assertEquals(42.0, turret.SupplyCurrentLimit, 1e-9);
    assertEquals(28.0, turret.SupplyCurrentLowerLimit, 1e-9);
    assertEquals(0.45, turret.SupplyCurrentLowerTime, 1e-9);

    var azimuth = Constants.SubsystemConstants.Turret.leftAzimuthConfig.CurrentLimits;
    assertEquals(45.0, azimuth.StatorCurrentLimit, 1e-9);
    assertEquals(18.0, azimuth.SupplyCurrentLimit, 1e-9);
    assertEquals(12.0, azimuth.SupplyCurrentLowerLimit, 1e-9);
    assertEquals(0.35, azimuth.SupplyCurrentLowerTime, 1e-9);

    var pivot = Constants.SubsystemConstants.Intake.pivotConfig.CurrentLimits;
    assertEquals(30.0, pivot.StatorCurrentLimit, 1e-9);
    assertEquals(14.0, pivot.SupplyCurrentLimit, 1e-9);
    assertEquals(10.0, pivot.SupplyCurrentLowerLimit, 1e-9);
    assertEquals(0.35, pivot.SupplyCurrentLowerTime, 1e-9);

    var indexer = Constants.SubsystemConstants.Indexer.indexerConfig.CurrentLimits;
    assertEquals(55.0, indexer.StatorCurrentLimit, 1e-9);
    assertEquals(35.0, indexer.SupplyCurrentLimit, 1e-9);
    assertEquals(24.0, indexer.SupplyCurrentLowerLimit, 1e-9);
    assertEquals(0.3, indexer.SupplyCurrentLowerTime, 1e-9);
  }

  @Test
  void battlecryDriveCurrentLimitsAreApplied() {
    var drive = TunerConstants.getDriveInitialConfigs().CurrentLimits;
    assertEquals(85.0, drive.StatorCurrentLimit, 1e-9);
    assertEquals(35.0, drive.SupplyCurrentLimit, 1e-9);
    assertEquals(25.0, drive.SupplyCurrentLowerLimit, 1e-9);
    assertEquals(0.35, drive.SupplyCurrentLowerTime, 1e-9);

    var steer = TunerConstants.getSteerInitialConfigs().CurrentLimits;
    assertEquals(40.0, steer.StatorCurrentLimit, 1e-9);
    assertEquals(18.0, steer.SupplyCurrentLimit, 1e-9);
    assertEquals(14.0, steer.SupplyCurrentLowerLimit, 1e-9);
    assertEquals(0.35, steer.SupplyCurrentLowerTime, 1e-9);
  }
}
