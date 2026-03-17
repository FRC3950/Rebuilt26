package frc.robot.controls;

import static frc.robot.Constants.SubsystemConstants.Intake.downPos;
import static frc.robot.Constants.SubsystemConstants.Intake.upPos;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import org.junit.jupiter.api.Test;

public class TuneModeBindingsTest {
  @Test
  void acceptsValidTuneSetpoint() {
    var setpoint = TuneModeBindings.validateTuneSetpoint(20.0, 50.0);

    assertTrue(setpoint.valid());
    assertEquals("OK", setpoint.status());
  }

  @Test
  void rejectsInvalidHoodAngle() {
    var setpoint = TuneModeBindings.validateTuneSetpoint(5.0, 50.0);

    assertFalse(setpoint.valid());
    assertEquals("Hood out of range", setpoint.status());
  }

  @Test
  void rejectsNegativeFlywheelSpeed() {
    var setpoint = TuneModeBindings.validateTuneSetpoint(20.0, -1.0);

    assertFalse(setpoint.valid());
    assertEquals("Flywheel must be non-negative", setpoint.status());
  }

  @Test
  void calculatesLeftTurretDistanceToHub() {
    double distance =
        TuneModeBindings.getDistanceToHub(
            Pose2d.kZero, frc.robot.Constants.SubsystemConstants.Turret.robotToTurret1);

    assertEquals(
        frc.robot.Constants.FieldConstants.hubTranslation.getDistance(
            frc.robot.Constants.SubsystemConstants.Turret.robotToTurret1),
        distance,
        1e-9);
  }

  @Test
  void togglesIntakeTowardOppositeLimit() {
    assertEquals(upPos, TuneModeBindings.getIntakeToggleTarget(downPos));
    assertEquals(downPos, TuneModeBindings.getIntakeToggleTarget(upPos));
  }
}
