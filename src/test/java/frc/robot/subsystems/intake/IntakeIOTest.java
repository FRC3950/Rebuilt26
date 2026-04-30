package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class IntakeIOTest {
  private static class FakeIntakeIO implements IntakeIO {
    double rollerVelocityRps = 0.0;
    double pivotPosition = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
      inputs.rollerVelocityRps = rollerVelocityRps;
      inputs.pivotPosition = pivotPosition;
    }

    @Override
    public void setRollerVelocity(double velocityRps) {
      rollerVelocityRps = velocityRps;
    }

    @Override
    public void setPivotPosition(double position) {
      pivotPosition = position;
    }
  }

  @Test
  void commandedAndMeasuredPivotStateAreSeparate() {
    FakeIntakeIO io = new FakeIntakeIO();
    Intake intake = new Intake(io);

    intake.extend();

    assertTrue(intake.isPivotCommandedDown());
    assertFalse(intake.isPivotMeasuredDown());

    intake.periodic();

    assertTrue(intake.isPivotMeasuredDown());
  }

  @Test
  void rollerMeasuredSpeedComesFromIoInputs() {
    FakeIntakeIO io = new FakeIntakeIO();
    Intake intake = new Intake(io);

    intake.startIntake();
    assertTrue(intake.isIntaking());
    assertFalse(intake.getMeasuredRollerSpeed() > 0.0);

    intake.periodic();

    assertTrue(intake.getMeasuredRollerSpeed() > 0.0);
  }
}
