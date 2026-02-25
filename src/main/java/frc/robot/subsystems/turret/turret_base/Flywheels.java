package frc.robot.subsystems.turret.turret_base;

import static frc.robot.Constants.SubsystemConstants.Turret.flywheelGearRatio;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class Flywheels {
  private final TalonFX flywheel;
  private final TalonFX flywheelFollower;

  private double lastSetpointRps = 0.0;
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  public Flywheels(
      int flywheelID, TalonFXConfiguration flywheelConfig, int flywheelFollowerID, CANBus canbus) {
    flywheel = new TalonFX(flywheelID, canbus);
    flywheelFollower = new TalonFX(flywheelFollowerID, canbus);

    flywheel.getConfigurator().apply(flywheelConfig);
    flywheelFollower.setControl(new Follower(flywheelID, MotorAlignmentValue.Opposed));
  }

  public void setTargetRps(double flywheelSpeedRps) {
    lastSetpointRps = flywheelSpeedRps;
    double targetVel = flywheelSpeedRps * flywheelGearRatio;
    flywheel.setControl(velocityVoltage.withVelocity(targetVel));
  }

  public void stop() {
    flywheel.set(0.0);
    lastSetpointRps = 0.0;
  }

  public void zeroPosition() {
    flywheel.setPosition(0.0);
  }

  public double getPositionRot() {
    return flywheel.getPosition().getValueAsDouble();
  }

  public double getVelocityRps() {
    return flywheel.getVelocity().getValueAsDouble();
  }

  public double getSetpointRps() {
    return lastSetpointRps;
  }

  public double getFollowerPositionRot() {
    return flywheelFollower.getPosition().getValueAsDouble();
  }

  public double getFollowerVelocityRps() {
    return flywheelFollower.getVelocity().getValueAsDouble();
  }
}
