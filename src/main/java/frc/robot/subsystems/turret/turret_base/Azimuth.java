package frc.robot.subsystems.turret.turret_base;

import static frc.robot.Constants.SubsystemConstants.Turret.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;

public class Azimuth {

  private final TalonFX azimuth;
  private final MotionMagicVoltage azimuthControl = new MotionMagicVoltage(0.0);

  private double lastSetpointDeg = 0.0;

  public Azimuth(int azimuthID, TalonFXConfiguration azimuthConfig, CANBus canbus) {
    azimuth = new TalonFX(azimuthID, canbus);
    azimuth.getConfigurator().apply(azimuthConfig);
  }

  public void setTargetAngleDeg(double targetAngleDeg) {
    lastSetpointDeg = targetAngleDeg;
    double motorRotations = Units.degreesToRotations(targetAngleDeg) * azimuthGearRatio;
    azimuth.setControl(azimuthControl.withPosition(motorRotations));
  }

  public double getMotorAngleDeg() {
    double azimuthRotations = azimuth.getPosition().getValueAsDouble();
    return Units.rotationsToDegrees(azimuthRotations / azimuthGearRatio);
  }

  public double getSetpointDeg() {
    return lastSetpointDeg;
  }
}
