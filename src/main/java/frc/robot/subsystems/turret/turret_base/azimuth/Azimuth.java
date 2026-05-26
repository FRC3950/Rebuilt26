package frc.robot.subsystems.turret.turret_base.azimuth;

import frc.robot.util.BatteryLogger;
import org.littletonrobotics.junction.Logger;

public class Azimuth {
  private final String logKey;
  private final AzimuthIO io;
  private final AzimuthIOInputsAutoLogged inputs = new AzimuthIOInputsAutoLogged();

  private double lastSetpointDeg = 0.0;
  private double lastVelocitySetpointDegPerSec = 0.0;

  public Azimuth(String logKey, AzimuthIO io) {
    this.logKey = logKey;
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(logKey, inputs);
    BatteryLogger.reportCurrentUsage(logKey, inputs.supplyCurrentAmps);
  }

  public void setTargetAngleDeg(double targetAngleDeg) {
    setTargetAngleDeg(targetAngleDeg, 0.0);
  }

  public void setTargetAngleDeg(double targetAngleDeg, double targetVelocityDegPerSec) {
    lastSetpointDeg = targetAngleDeg;
    lastVelocitySetpointDegPerSec = targetVelocityDegPerSec;
    io.setTargetAngleDeg(targetAngleDeg, targetVelocityDegPerSec);
  }

  public double getMotorAngleDeg() {
    return inputs.positionDeg;
  }

  public double getMeasuredAngleDeg() {
    return inputs.positionDeg;
  }

  public double getVelocityDegPerSec() {
    return inputs.velocityDegPerSec;
  }

  public double getSetpointDeg() {
    return lastSetpointDeg;
  }

  public double getVelocitySetpointDegPerSec() {
    return lastVelocitySetpointDegPerSec;
  }

  public boolean isZeroSwitchClosed() {
    return inputs.zeroSwitchClosed;
  }

  public void zeroPosition() {
    io.zeroPosition();
    lastSetpointDeg = 0.0;
    lastVelocitySetpointDegPerSec = 0.0;
  }
}
