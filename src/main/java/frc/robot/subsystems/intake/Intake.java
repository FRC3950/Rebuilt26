// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.Constants.SubsystemConstants.Intake.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BatteryLogger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private double commandedRollerSpeed = 0.0;
  private double pivotSetpoint = upPos;
  private boolean isIntaking = false;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    BatteryLogger.reportCurrentUsage("Intake/Roller", inputs.rollerSupplyCurrentAmps);
    BatteryLogger.reportCurrentUsage("Intake/Pivot", inputs.pivotSupplyCurrentAmps);
  }

  public void setIntakeSpeed(double speed) {
    commandedRollerSpeed = speed;
    isIntaking = speed > 0.0;
    io.setRollerVelocity(speed);
  }

  public void reverseIntake() {
    setIntakeSpeed(unjamSpeed);
  }

  public void startIntake() {
    isIntaking = true;
    setIntakeSpeed(mintakeSpeed + 8);
  }

  public void stopIntake() {
    isIntaking = false;
    setIntakeSpeed(0);
  }

  public boolean isIntaking() {
    return isIntaking;
  }

  @AutoLogOutput(key = "Intake/Current Speed")
  public double getIntakeCurrentSpeed() {
    return inputs.rollerVelocityRps;
  }

  public double getMeasuredRollerSpeed() {
    return inputs.rollerVelocityRps;
  }

  public void setPivotPosition(double position) {
    pivotSetpoint = position;
    io.setPivotPosition(position);
  }

  public void extend() {
    setPivotPosition(downPos);
  }

  public void retract() {
    setPivotPosition(upPos);
  }

  public boolean isAcceptablePosition(double targetPos) {
    return Math.abs(inputs.pivotPosition - targetPos) < 0.05;
  }

  @AutoLogOutput(key = "Intake/Pivot Position")
  public double getPivotPosition() {
    return inputs.pivotPosition;
  }

  public double getCommandedRollerSpeed() {
    return commandedRollerSpeed;
  }

  @AutoLogOutput(key = "Intake/Pivot Setpoint")
  public double getPivotSetpoint() {
    return pivotSetpoint;
  }

  @AutoLogOutput(key = "Intake/Commanded Roller Speed")
  public double getLoggedCommandedRollerSpeed() {
    return getCommandedRollerSpeed();
  }

  public boolean isPivotCommandedDown() {
    return Math.abs(pivotSetpoint - downPos) < 1e-9;
  }

  public boolean isPivotMeasuredDown() {
    return isAcceptablePosition(downPos);
  }

  public Command extendCommand() {
    return this.runOnce(this::extend);
  }

  public Command retractCommand() {
    return this.runOnce(this::retract);
  }

  public Command onIntake() {
    return this.runOnce(this::startIntake);
  }

  public Command zeroIntake() {
    return Commands.runOnce(io::zeroPivotPosition, this);
  }

  public Command offIntake() {
    return this.runOnce(this::stopIntake);
  }
}
