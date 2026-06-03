package frc.robot.sim;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

public class FuelSimCommand extends Command {
  private final FuelSimulationController controller;
  private final BooleanSupplier unlimitedFuelCapacitySupplier;

  public FuelSimCommand(FuelSimulationController controller) {
    this(controller, () -> false);
  }

  public FuelSimCommand(
      FuelSimulationController controller, BooleanSupplier unlimitedFuelCapacitySupplier) {
    this.controller = controller;
    this.unlimitedFuelCapacitySupplier = unlimitedFuelCapacitySupplier;
  }

  @Override
  public void initialize() {
    controller.initializeSimulation();
  }

  @Override
  public void execute() {
    controller.setUnlimitedFuelCapacityForSim(unlimitedFuelCapacitySupplier.getAsBoolean());
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
