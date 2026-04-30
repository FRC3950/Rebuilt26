package frc.robot.subsystems.turret.turret_base.flywheels;

import static frc.robot.Constants.SubsystemConstants.Turret.maxFlywheelRps;
import static frc.robot.Constants.SubsystemConstants.Turret.minFlywheelRps;

import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class Flywheels {
  private final String logKey;
  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

  private double targetRps = 0.0;

  public Flywheels(String logKey, FlywheelsIO io) {
    this.logKey = logKey;
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(logKey, inputs);
  }

  public void setTargetRps(double flywheelSpeedRps) {
    targetRps = MathUtil.clamp(flywheelSpeedRps, minFlywheelRps, maxFlywheelRps);
    io.setTargetRps(targetRps);
  }

  public double getVelocityRps() {
    return inputs.leaderVelocityRps;
  }

  public double getMeasuredVelocityRps() {
    return inputs.leaderVelocityRps;
  }

  public double getFollowerVelocityRps() {
    return inputs.followerVelocityRps;
  }

  public double getTargetRps() {
    return targetRps;
  }
}
