// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

import static frc.robot.Constants.SubsystemConstants.Intake.mintakeSpeed;

import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {
  private final Intake intake;
  private final DoubleSupplier driveSpeedMetersPerSecond;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake intake, DoubleSupplier driveSpeedMetersPerSecond) {
    this.intake = intake;
    this.driveSpeedMetersPerSecond = driveSpeedMetersPerSecond;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.extend();
    intake.setIntakeSpeed(mintakeSpeed + 3 * driveSpeedMetersPerSecond.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
