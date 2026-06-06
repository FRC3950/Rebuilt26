package frc.robot.subsystems.turret.turret_base;

import static frc.robot.Constants.SubsystemConstants.Turret.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.BatteryLogger;
import org.littletonrobotics.junction.Logger;

public class Azimuth {
  private static final int POWER_STATUS_PERIOD_LOOPS = 5;

  private final TalonFX azimuth;
  private final PositionVoltage azimuthControl = new PositionVoltage(0.0).withUpdateFreqHz(250);
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Voltage> supplyVoltage;
  private final StatusSignal<Current> supplyCurrent;

  private double lastSetpointDeg = 0.0;
  private double lastVelocitySetpointDegPerSec = 0.0;
  private int refreshCounter = 0;

  public Azimuth(int azimuthID, TalonFXConfiguration azimuthConfig, CANBus canbus) {
    azimuth = new TalonFX(azimuthID, canbus);
    position = azimuth.getPosition();
    velocity = azimuth.getVelocity();
    appliedVolts = azimuth.getMotorVoltage();
    current = azimuth.getStatorCurrent();
    supplyVoltage = azimuth.getSupplyVoltage();
    supplyCurrent = azimuth.getSupplyCurrent();
    azimuth.getConfigurator().apply(azimuthConfig);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, supplyVoltage, supplyCurrent);
    ParentDevice.optimizeBusUtilizationForAll(azimuth);
  }

  public void periodic(String logKey) {
    boolean refreshPowerSignals = refreshCounter % POWER_STATUS_PERIOD_LOOPS == 0;
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);
    if (refreshPowerSignals) {
      BaseStatusSignal.refreshAll(supplyVoltage, supplyCurrent);
    }

    Logger.recordOutput(logKey + "/PositionDeg", getMeasuredAngleDeg());
    Logger.recordOutput(logKey + "/VelocityDegPerSec", getVelocityDegPerSec());
    Logger.recordOutput(logKey + "/AppliedVolts", appliedVolts.getValueAsDouble());
    Logger.recordOutput(logKey + "/CurrentAmps", current.getValueAsDouble());
    if (refreshPowerSignals) {
      Logger.recordOutput(logKey + "/SupplyVoltageVolts", supplyVoltage.getValueAsDouble());
      Logger.recordOutput(logKey + "/SupplyCurrentAmps", supplyCurrent.getValueAsDouble());
    }
    BatteryLogger.reportCurrentUsage(logKey, supplyCurrent.getValueAsDouble());
    refreshCounter++;
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
    double azimuthRotations = position.getValueAsDouble();
    return Units.rotationsToDegrees(azimuthRotations / azimuthGearRatio);
  }

  public double getMeasuredAngleDeg() {
    return getMotorAngleDeg();
  }

  public double getVelocityDegPerSec() {
    return Units.rotationsToDegrees(velocity.getValueAsDouble() / azimuthGearRatio);
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
