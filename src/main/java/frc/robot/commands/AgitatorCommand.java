package frc.robot.commands;

import static frc.robot.Constants.SubsystemConstants.Intake.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class AgitatorCommand extends Command {
  private final Intake intake;
  private boolean isExtended = false;

  public AgitatorCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.extend();
    isExtended = true;
  }

  @Override
  public void execute() {
    if (intake.isIntaking()) {
      return;
    }

    if (isExtended) {
      if (intake.isAcceptablePosition(downPos)) {
        intake.retract();
        isExtended = false;
      }
    } else {
      if (intake.isAcceptablePosition(upPos)) {
        intake.extend();
        isExtended = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.extend();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
