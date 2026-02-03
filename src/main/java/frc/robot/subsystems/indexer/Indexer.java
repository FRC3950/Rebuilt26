// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static frc.robot.Constants.SubsystemConstants.CANivore;
import static frc.robot.Constants.SubsystemConstants.Indexer.*;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Indexer extends SubsystemBase {

  TalonFX hotdogMotor;
  TalonFX indexerMotor;

  private final VelocityVoltage indexerControl = new VelocityVoltage(0);
  private final VelocityVoltage hotdogControl = new VelocityVoltage(0);

  public Indexer() {
    hotdogMotor = new TalonFX(hotdogMotorID, CANivore);
    indexerMotor = new TalonFX(indexerMotorID, CANivore);

    hotdogMotor.getConfigurator().apply(hotdogConfig);
    indexerMotor.getConfigurator().apply(indexerConfig);
  }

  public void setIndexerSpeed(double speed) {
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
    return indexerMotor.getVelocity().getValueAsDouble();
  }

  public void setHotdogSpeed(double speed) {
    hotdogMotor.setControl(hotdogControl.withVelocity(speed));
  }

  public void startHotdog() {
    setHotdogSpeed(indexerSpeed);
  }

  public void stopHotdog() {
    setHotdogSpeed(0);
  }

  @AutoLogOutput(key = "Indexer/Hotdog Speed")
  public double getHotdogCurrentSpeed() {
    return hotdogMotor.getVelocity().getValueAsDouble();
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
