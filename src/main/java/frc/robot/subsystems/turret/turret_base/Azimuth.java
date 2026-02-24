package frc.robot.subsystems.turret.turret_base;

import static frc.robot.Constants.SubsystemConstants.Turret.azimuthGearRatio;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.turret.CRT;
import java.util.OptionalDouble;

public class Azimuth {
  private final TalonFX azimuth;
  private final CANcoder encoderA;
  private final CANcoder encoderB;
  private final MotionMagicVoltage azimuthControl = new MotionMagicVoltage(0.0);
  private final CRT.Parameters crtParameters;

  private double lastSetpointDeg = 0.0;

  public Azimuth(
      int azimuthID,
      TalonFXConfiguration azimuthConfig,
      int encoderAID,
      int encoderBID,
      CANBus canbus,
      CRT.Parameters crtParameters) {
    azimuth = new TalonFX(azimuthID, canbus);
    encoderA = new CANcoder(encoderAID, canbus);
    encoderB = new CANcoder(encoderBID, canbus);
    this.crtParameters = crtParameters;

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

  public double getMotorVelocityDegPerSec() {
    return Units.rotationsToDegrees(azimuth.getVelocity().getValueAsDouble() / azimuthGearRatio);
  }

  public double getSetpointDeg() {
    return lastSetpointDeg;
  }

  public double getEncoderAAbsRot() {
    return encoderA.getAbsolutePosition().getValueAsDouble();
  }

  public double getEncoderBAbsRot() {
    return encoderB.getAbsolutePosition().getValueAsDouble();
  }

  public OptionalDouble getCrtAbsoluteAngleDegOptional() {
    return CRT.solveAbsoluteAngleDeg(getEncoderAAbsRot(), getEncoderBAbsRot(), crtParameters);
  }

  public double getCrtAbsoluteAngleDeg() {
    return getCrtAbsoluteAngleDegOptional().orElse(Double.NaN);
  }

  public boolean isCrtAbsoluteAngleValid() {
    return getCrtAbsoluteAngleDegOptional().isPresent();
  }

  public boolean syncMotorPositionToCrtIfValid() {
    OptionalDouble absoluteAngleDeg = getCrtAbsoluteAngleDegOptional();
    if (absoluteAngleDeg.isEmpty()) {
      return false;
    }

    double syncedAngleDeg = absoluteAngleDeg.getAsDouble();
    azimuth.setPosition(Units.degreesToRotations(syncedAngleDeg) * azimuthGearRatio);
    lastSetpointDeg = syncedAngleDeg;
    return true;
  }

  public void zeroMotorPosition() {
    azimuth.setPosition(0.0);
    lastSetpointDeg = 0.0;
  }
}
