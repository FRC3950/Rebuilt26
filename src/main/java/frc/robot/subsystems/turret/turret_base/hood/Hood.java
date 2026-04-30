package frc.robot.subsystems.turret.turret_base.hood;

import static frc.robot.Constants.SubsystemConstants.Turret.maxHoodAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.minHoodAngle;

import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class Hood {
  private final String logKey;
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private double lastSetpointDeg = minHoodAngle;

  public Hood(String logKey, HoodIO io) {
    this.logKey = logKey;
    this.io = io;
    setAngleDeg(minHoodAngle);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(logKey, inputs);
  }

  public void setAngleDeg(double hoodAngleDeg) {
    lastSetpointDeg = MathUtil.clamp(hoodAngleDeg, minHoodAngle, maxHoodAngle);
    io.setAngleDeg(lastSetpointDeg);
  }

  public double getPositionDeg() {
    return inputs.positionDeg;
  }

  public double getSetpointDeg() {
    return lastSetpointDeg;
  }
}
