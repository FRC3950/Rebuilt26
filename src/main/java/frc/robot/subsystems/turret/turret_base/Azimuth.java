package frc.robot.subsystems.turret.turret_base;

import static frc.robot.Constants.SubsystemConstants.Turret.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;

public class Azimuth {
  private final TalonFX azimuth;
  private final PositionVoltage azimuthControl = new PositionVoltage(0.0).withUpdateFreqHz(250);

  private double lastSetpointDeg = 0.0;
  private double lastVelocitySetpointDegPerSec = 0.0;

  public Azimuth(int azimuthID, TalonFXConfiguration azimuthConfig, CANBus canbus) {
    azimuth = new TalonFX(azimuthID, canbus);
    azimuth.getConfigurator().apply(azimuthConfig);
  }

  public void setTargetAngleDeg(double targetAngleDeg) {
    setTargetAngleDeg(targetAngleDeg, 0.0);
  }

  public void setTargetAngleDeg(double targetAngleDeg, double targetVelocityDegPerSec) {
    lastSetpointDeg = targetAngleDeg;
    lastVelocitySetpointDegPerSec = targetVelocityDegPerSec;
    double motorRotations = Units.degreesToRotations(targetAngleDeg) * azimuthGearRatio;
    double motorRotationsPerSecond =
        Units.degreesToRotations(targetVelocityDegPerSec) * azimuthGearRatio;
    azimuth.setControl(
        azimuthControl.withPosition(motorRotations).withVelocity(motorRotationsPerSecond));
  }

  public double getMotorAngleDeg() {
    double azimuthRotations = azimuth.getPosition().getValueAsDouble();
    return Units.rotationsToDegrees(azimuthRotations / azimuthGearRatio);
  }

  public double getMeasuredAngleDeg() {
    return getMotorAngleDeg();
  }

  public double getSetpointDeg() {
    return lastSetpointDeg;
  }

  public double getVelocitySetpointDegPerSec() {
    return lastVelocitySetpointDegPerSec;
  }

  public void zeroPosition() {
    azimuth.setPosition(0.0);
    lastSetpointDeg = 0.0;
    lastVelocitySetpointDegPerSec = 0.0;
  }
}
