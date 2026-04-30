package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public boolean rollerConnected = false;
    public double rollerVelocityRps = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;

    public boolean pivotConnected = false;
    public double pivotPosition = 0.0;
    public double pivotVelocityRps = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setRollerVelocity(double velocityRps) {}

  default void setPivotPosition(double position) {}

  default void zeroPivotPosition() {}
}
