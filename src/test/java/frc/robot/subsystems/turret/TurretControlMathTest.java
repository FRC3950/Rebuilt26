package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

class TurretControlMathTest {
  @Test
  void autoTargetAddsVelocityLookaheadToRequestedAzimuth() {
    double turretVelocityRadPerSec = 1.5;

    double targetDeg =
        Turret.getVelocityCompensatedTargetDeg(
            Rotation2d.fromDegrees(10.0), turretVelocityRadPerSec, 0.0);

    assertEquals(10.0 + Units.radiansToDegrees(turretVelocityRadPerSec) * 0.02, targetDeg, 1e-9);
  }

  @Test
  void trimIsAppliedBeforeSafeSetpointSelection() {
    assertEquals(
        11.25,
        Turret.getVelocityCompensatedTargetDeg(Rotation2d.fromDegrees(10.0), 0.0, 1.25),
        1e-9);
  }
}
