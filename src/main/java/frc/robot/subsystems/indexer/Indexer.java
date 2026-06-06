// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static frc.robot.Constants.SubsystemConstants.CANivore;
import static frc.robot.Constants.SubsystemConstants.Indexer.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BatteryLogger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private static final int POWER_STATUS_PERIOD_LOOPS = 5;

  private final TalonFX hotdogMotor;
  private final TalonFX indexerMotor;

  private final VelocityVoltage indexerControl = new VelocityVoltage(0);
  private final VelocityVoltage hotdogControl = new VelocityVoltage(0);
  private final StatusSignal<AngularVelocity> indexerVelocity;
  private final StatusSignal<Voltage> indexerAppliedVolts;
  private final StatusSignal<Current> indexerCurrent;
  private final StatusSignal<Voltage> indexerSupplyVoltage;
  private final StatusSignal<Current> indexerSupplyCurrent;
  private final StatusSignal<AngularVelocity> hotdogVelocity;
  private final StatusSignal<Voltage> hotdogAppliedVolts;
  private final StatusSignal<Current> hotdogCurrent;
  private final StatusSignal<Voltage> hotdogSupplyVoltage;
  private final StatusSignal<Current> hotdogSupplyCurrent;
  private double commandedIndexerSpeed = 0.0;
  private double commandedHotdogSpeed = 0.0;
  private int refreshCounter = 0;

  public Indexer() {
    hotdogMotor = new TalonFX(hotdogMotorID, CANivore);
    indexerMotor = new TalonFX(indexerMotorID, CANivore);
    indexerVelocity = indexerMotor.getVelocity();
    indexerAppliedVolts = indexerMotor.getMotorVoltage();
    indexerCurrent = indexerMotor.getStatorCurrent();
    indexerSupplyVoltage = indexerMotor.getSupplyVoltage();
    indexerSupplyCurrent = indexerMotor.getSupplyCurrent();
    hotdogVelocity = hotdogMotor.getVelocity();
    hotdogAppliedVolts = hotdogMotor.getMotorVoltage();
    hotdogCurrent = hotdogMotor.getStatorCurrent();
    hotdogSupplyVoltage = hotdogMotor.getSupplyVoltage();
    hotdogSupplyCurrent = hotdogMotor.getSupplyCurrent();

    hotdogMotor.getConfigurator().apply(hotdogConfig);
    indexerMotor.getConfigurator().apply(indexerConfig);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        indexerVelocity,
        indexerAppliedVolts,
        indexerCurrent,
        hotdogVelocity,
        hotdogAppliedVolts,
        hotdogCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0, indexerSupplyVoltage, indexerSupplyCurrent, hotdogSupplyVoltage, hotdogSupplyCurrent);
    ParentDevice.optimizeBusUtilizationForAll(hotdogMotor, indexerMotor);
  }

  @Override
  public void periodic() {
    boolean refreshPowerSignals = refreshCounter % POWER_STATUS_PERIOD_LOOPS == 0;
    BaseStatusSignal.refreshAll(indexerVelocity, indexerAppliedVolts, indexerCurrent);
    BaseStatusSignal.refreshAll(hotdogVelocity, hotdogAppliedVolts, hotdogCurrent);
    if (refreshPowerSignals) {
      BaseStatusSignal.refreshAll(
          indexerSupplyVoltage, indexerSupplyCurrent, hotdogSupplyVoltage, hotdogSupplyCurrent);
    }

    Logger.recordOutput("Indexer/Indexer Applied Volts", indexerAppliedVolts.getValueAsDouble());
    Logger.recordOutput("Indexer/Indexer Current Amps", indexerCurrent.getValueAsDouble());
    Logger.recordOutput("Indexer/Hotdog Applied Volts", hotdogAppliedVolts.getValueAsDouble());
    Logger.recordOutput("Indexer/Hotdog Current Amps", hotdogCurrent.getValueAsDouble());
    if (refreshPowerSignals) {
      Logger.recordOutput(
          "Indexer/Indexer Supply Voltage", indexerSupplyVoltage.getValueAsDouble());
      Logger.recordOutput(
          "Indexer/Indexer Supply Current", indexerSupplyCurrent.getValueAsDouble());
      Logger.recordOutput("Indexer/Hotdog Supply Voltage", hotdogSupplyVoltage.getValueAsDouble());
      Logger.recordOutput("Indexer/Hotdog Supply Current", hotdogSupplyCurrent.getValueAsDouble());
    }
    BatteryLogger.reportCurrentUsage("Indexer/Indexer", indexerSupplyCurrent.getValueAsDouble());
    BatteryLogger.reportCurrentUsage("Indexer/Hotdog", hotdogSupplyCurrent.getValueAsDouble());
    refreshCounter++;
  }

  public void setIndexerSpeed(double speed) {
    commandedIndexerSpeed = speed;
    indexerMotor.setControl(indexerControl.withVelocity(speed));
  }

  public void startIndexer() {
    setIndexerSpeed(indexerSpeed);
  }

  public void stopIndexer() {
    setIndexerSpeed(0);
  }

  @AutoLogOutput(key = "Indexer/Indexer Speed")
  public double getIndexerCurrentSpeed() {
    return indexerVelocity.getValueAsDouble();
  }

  public void setHotdogSpeed(double speed) {
    commandedHotdogSpeed = speed;
    hotdogMotor.setControl(hotdogControl.withVelocity(speed));
  }

  public void startHotdog() {
    setHotdogSpeed(hotdogSpeed);
  }

  public void reverseHotdog() {
    setHotdogSpeed(unjamHotdog);
  }

  public void stopHotdog() {
    setHotdogSpeed(0);
  }

  @AutoLogOutput(key = "Indexer/Hotdog Speed")
  public double getHotdogCurrentSpeed() {
    return hotdogVelocity.getValueAsDouble();
  }

  public double getCommandedIndexerSpeed() {
    return commandedIndexerSpeed;
  }

  @AutoLogOutput(key = "Indexer/Commanded Hotdog Speed")
  public double getCommandedHotdogSpeed() {
    return commandedHotdogSpeed;
  }

  @AutoLogOutput(key = "Indexer/Commanded Indexer Speed")
  public double getLoggedCommandedIndexerSpeed() {
    return getCommandedIndexerSpeed();
  }

  @AutoLogOutput(key = "Indexer/Feeding Forward")
  public boolean isFeedingForward() {
    return commandedIndexerSpeed > 0.0 && commandedHotdogSpeed > 0.0;
  }

  public Command feedCommand() {
    return this.runEnd(
        () -> {
          startIndexer();
          startHotdog();
        },
        () -> {
          stopIndexer();
          stopHotdog();
        });
  }

  public Command runEndHotdog(double speed) {
    return this.runEnd(() -> setHotdogSpeed(speed), () -> stopHotdog());
  }
}
