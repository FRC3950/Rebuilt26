// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static frc.robot.Constants.SubsystemConstants.Climber.*;
import static frc.robot.Constants.SubsystemConstants.CANivore;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Climber extends SubsystemBase {
  
  private final TalonFX climberMotor;
  private final DigitalInput limitSwitch;
  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

  public Climber() {
    climberMotor = new TalonFX(climberMotorID, CANivore);
    limitSwitch = new DigitalInput(limitSwitchPort);
    climberMotor.getConfigurator().apply(climberConfig);

    // Reset position to 0 when limit switch is triggered
    new Trigger(limitSwitch::get).onTrue(runOnce(() -> climberMotor.setPosition(0)));
  }

  public void setPosition(double targetPosition) {
    climberMotor.setControl(mmRequest.withPosition(targetPosition));
  }

  public Command togglePositionCommand() {
    return runOnce(() -> {
      double currentPos = climberMotor.getPosition().getValueAsDouble();
      double target = (Math.abs(currentPos) < climbUpHeight / 2.0) ? climbUpHeight : 0.0;
      setPosition(target);
    });
  }
}
