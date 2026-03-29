package frc.robot.sim;

import edu.wpi.first.wpilibj2.command.Command;

public class FuelSimCommand extends Command {
  private final FuelSimulationController controller;

  public FuelSimCommand(FuelSimulationController controller) {
    this.controller = controller;
  }

  @Override
  public void initialize() {
    controller.initializeSimulation();
  }

  @Override
  public void execute() {
    controller.stepSimulation();
  }

  @Override
  public void end(boolean interrupted) {
    controller.stopSimulation();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
