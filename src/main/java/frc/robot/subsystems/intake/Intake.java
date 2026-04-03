// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.Constants.SubsystemConstants.CANivore;
import static frc.robot.Constants.SubsystemConstants.Intake.*;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Intake extends SubsystemBase {
  private final TalonFX intakeMotor;
  private final TalonFX pivotMotor;
  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);
  private final VelocityVoltage intakeControlRequest = new VelocityVoltage(0);
  private double commandedRollerSpeed = 0.0;
  private double pivotSetpoint = upPos;
  private boolean isIntaking = false;

  /** Creates a new intake. */
  public Intake() {
    intakeMotor = new TalonFX(intakeMotorID, CANivore);
    pivotMotor = new TalonFX(pivotMotorID, CANivore);

    pivotMotor.getConfigurator().apply(pivotConfig);
    intakeMotor.getConfigurator().apply(intakeConfig);
  }

  public void setIntakeSpeed(double speed) {
    commandedRollerSpeed = speed;
    isIntaking = speed > 0.0;
    intakeMotor.setControl(intakeControlRequest.withVelocity(speed));
  }

  public void reverseIntake(){
    intakeMotor.setControl(intakeControlRequest.withVelocity(unjamSpeed));
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
    return intakeMotor.getVelocity().getValueAsDouble();
  }

  public void setPivotPosition(double position) {
    pivotSetpoint = position;
    pivotMotor.setControl(mmRequest.withPosition(position));
  }

  public void extend() {
    setPivotPosition(downPos);
  }

  public void retract() {
    setPivotPosition(upPos);
  }

  public boolean isAcceptablePosition(double targetPos) {
    return Math.abs(pivotMotor.getPosition().getValueAsDouble() - targetPos) < 0.05;
  }

  @AutoLogOutput(key = "Intake/Pivot Position")
  public double getPivotPosition() {
    return pivotMotor.getPosition().getValueAsDouble();
  }

  public double getCommandedRollerSpeed() {
    return commandedRollerSpeed;
  }

  public double getPivotSetpoint() {
    return pivotSetpoint;
  }

  public boolean isPivotCommandedDown() {
    return Math.abs(pivotSetpoint - downPos) < 1e-9;
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
    return Commands.runOnce(() -> pivotMotor.setPosition(0), this);
  }

  public Command offIntake() {
    return this.runOnce(this::stopIntake);
  }
}
