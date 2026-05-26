package frc.robot.subsystems.turret.turret_base.azimuth;

import org.littletonrobotics.junction.AutoLog;

public interface AzimuthIO {
  @AutoLog
  class AzimuthIOInputs {
    public boolean connected = false;
    public double positionDeg = 0.0;
    public double velocityDegPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double supplyVoltageVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public boolean zeroSwitchClosed = false;
  }

  default void updateInputs(AzimuthIOInputs inputs) {}

  default void setTargetAngleDeg(double targetAngleDeg) {}

  default void setTargetAngleDeg(double targetAngleDeg, double targetVelocityDegPerSec) {
    setTargetAngleDeg(targetAngleDeg);
  }

  default void zeroPosition() {}
}
