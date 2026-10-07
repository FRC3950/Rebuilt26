// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static frc.robot.Constants.SubsystemConstants.CANivore;
import static frc.robot.Constants.SubsystemConstants.Indexer.*;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Indexer extends SubsystemBase {

  TalonFX hotdogMotor;
  TalonFX indexerMotor;

  private final VelocityVoltage indexerControl = new VelocityVoltage(0);
  private final VelocityVoltage hotdogControl = new VelocityVoltage(0);
  private double commandedIndexerSpeed = 0.0;
  private double commandedHotdogSpeed = 0.0;
  private double requestedIndexerSpeed = 0.0;
  private double requestedHotdogSpeed = 0.0;
  private BooleanSupplier forwardFeedAllowed = () -> true;

  public Indexer() {
    hotdogMotor = new TalonFX(hotdogMotorID, CANivore);
    indexerMotor = new TalonFX(indexerMotorID, CANivore);

    hotdogMotor.getConfigurator().apply(hotdogConfig);
    indexerMotor.getConfigurator().apply(indexerConfig);
  }

  public void setIndexerSpeed(double speed) {
    requestedIndexerSpeed = speed;
    updateOutputs();
  }

  public void startIndexer() {
    setIndexerSpeed(indexerSpeed);
  }

  public void stopIndexer() {
    setIndexerSpeed(0);
  }

  @AutoLogOutput(key = "Indexer/Indexer Speed")
  public double getIndexerCurrentSpeed() {
    return indexerMotor.getVelocity().getValueAsDouble();
  }

  public void setHotdogSpeed(double speed) {
    requestedHotdogSpeed = speed;
    updateOutputs();
  }

  public void setForwardFeedInterlock(BooleanSupplier forwardFeedAllowed) {
    this.forwardFeedAllowed = forwardFeedAllowed;
  }

  @Override
  public void periodic() {
    updateOutputs();
  }

  /** Recheck latched feed requests each tick and immediately when a turret starts flipping. */
  public void updateOutputs() {
    if (DriverStation.isDisabled()) {
      requestedIndexerSpeed = 0.0;
      requestedHotdogSpeed = 0.0;
    }
    boolean allowed = forwardFeedAllowed.getAsBoolean();
    commandedIndexerSpeed = permittedSpeed(requestedIndexerSpeed, allowed);
    commandedHotdogSpeed = permittedSpeed(requestedHotdogSpeed, allowed);
    indexerMotor.setControl(indexerControl.withVelocity(commandedIndexerSpeed));
    hotdogMotor.setControl(hotdogControl.withVelocity(commandedHotdogSpeed));
  }

  static double permittedSpeed(double requestedSpeed, boolean forwardAllowed) {
    return !forwardAllowed && requestedSpeed > 0.0 ? 0.0 : requestedSpeed;
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
    return hotdogMotor.getVelocity().getValueAsDouble();
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
