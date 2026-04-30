package frc.robot.subsystems.turret.turret_base.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {
  @AutoLog
  class FlywheelsIOInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;
    public double leaderVelocityRps = 0.0;
    public double followerVelocityRps = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(FlywheelsIOInputs inputs) {}

  default void setTargetRps(double flywheelSpeedRps) {}
}
