package frc.robot.subsystems.turret.turret_base.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  class HoodIOInputs {
    public boolean connected = false;
    public double positionDeg = 0.0;
    public double pulseWidthUs = 0.0;
    public double hubDeviceVoltageVolts = 0.0;
    public double hubDeviceCurrentAmps = 0.0;
    public double servoVoltageVolts = 0.0;
    public double channelCurrentAmps = 0.0;
    public boolean hubHasActiveFault = false;
    public boolean hubHasActiveWarning = false;
  }

  default void updateInputs(HoodIOInputs inputs) {}

  default void setAngleDeg(double hoodAngleDeg) {}
}
