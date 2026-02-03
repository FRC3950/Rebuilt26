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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Intake extends SubsystemBase {
  private final TalonFX intakeMotor;
  private final TalonFX pivotMotor;
  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);
  private final VelocityVoltage intakeControlRequest = new VelocityVoltage(0);

  /** Creates a new intake. */
  public Intake() {
    intakeMotor = new TalonFX(intakeMotorID, CANivore);
    pivotMotor = new TalonFX(pivotMotorID, CANivore);

    pivotMotor.getConfigurator().apply(pivotConfig);
    intakeMotor.getConfigurator().apply(intakeConfig);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.setControl(intakeControlRequest.withVelocity(speed));
  }

  public void startIntake() {
    setIntakeSpeed(intakeSpeed);
  }

  public void stopIntake() {
    setIntakeSpeed(0);
  }

  @AutoLogOutput(key = "Intake/Current Speed")
  public double getIntakeCurrentSpeed() {
    return intakeMotor.getVelocity().getValueAsDouble();
  }

  public void setPivotPosition(double position) {
    pivotMotor.setControl(mmRequest.withPosition(position));
  }

  public void extend() {
    pivotMotor.setControl(mmRequest.withPosition(downPos));
  }

  public void retract() {
    pivotMotor.setControl(mmRequest.withPosition(upPos));
  }

  @AutoLogOutput(key = "Intake/Pivot Position")
  public double getPivotPosition() {
    return pivotMotor.getPosition().getValueAsDouble();
  }

  public Command retractCommand() {
    return this.runOnce(this::retract);
  }
}
