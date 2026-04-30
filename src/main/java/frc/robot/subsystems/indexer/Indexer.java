// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static frc.robot.Constants.SubsystemConstants.Indexer.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private double commandedIndexerSpeed = 0.0;
  private double commandedHotdogSpeed = 0.0;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  public void setIndexerSpeed(double speed) {
    commandedIndexerSpeed = speed;
    io.setIndexerVelocity(speed);
  }

  public void startIndexer() {
    setIndexerSpeed(indexerSpeed);
  }

  public void stopIndexer() {
    setIndexerSpeed(0);
  }

  @AutoLogOutput(key = "Indexer/Indexer Speed")
  public double getIndexerCurrentSpeed() {
    return inputs.indexerVelocityRps;
  }

  public void setHotdogSpeed(double speed) {
    commandedHotdogSpeed = speed;
    io.setHotdogVelocity(speed);
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
    return inputs.hotdogVelocityRps;
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

  public boolean isMeasuredFeedingForward() {
    return inputs.indexerVelocityRps > 0.0 && inputs.hotdogVelocityRps > 0.0;
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
