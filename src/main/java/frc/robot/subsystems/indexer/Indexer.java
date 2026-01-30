// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static frc.robot.Constants.SubsystemConstants.CANivore;
import static frc.robot.Constants.SubsystemConstants.Indexer.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  TalonFX hotdogMotor;
  TalonFX indexerMotor;

  public Indexer() {
    hotdogMotor = new TalonFX(hotdogMotorID, CANivore);
    indexerMotor = new TalonFX(indexerMotorID, CANivore);
  }

  public void setIndexerSpeed(double speed) {
    indexerMotor.set(speed);
  }

  public void startIndexer() {
    setIndexerSpeed(indexerSpeed);
  }

  public void stopIndexer() {
    setIndexerSpeed(0);
  }

  public void setHotdogSpeed(double speed) {
    hotdogMotor.set(speed);
  }

  public void startHotdog() {
    setHotdogSpeed(indexerSpeed);
  }

  public void stopHotdog() {
    setHotdogSpeed(0);
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
    return this.runEnd(
        () -> setHotdogSpeed(speed),
        () -> stopHotdog());
  }
}
