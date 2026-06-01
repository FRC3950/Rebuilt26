package frc.robot.controls.sim;

import frc.robot.DemoContainer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RobotSimulationControls {
  private final Runnable enableDemoMode;
  private final Supplier<DemoContainer> demoContainerSupplier;

  public RobotSimulationControls(
      Runnable enableDemoMode, Supplier<DemoContainer> demoContainerSupplier) {
    this.enableDemoMode = enableDemoMode;
    this.demoContainerSupplier = demoContainerSupplier;
  }

  public void enableDemoMode() {
    enableDemoMode.run();
  }

  public void setHandheldTagTargetingEnabled(boolean enabled) {
    getOrEnableDemoContainer().setHandheldTagTargetingEnabled(enabled);
  }

  public void setTargetTagId(int tagId) {
    getOrEnableDemoContainer().setTargetTagId(tagId);
  }

  public void setPose(double xMeters, double yMeters, double headingDeg) {
    getOrEnableDemoContainer().setSimPose(xMeters, yMeters, headingDeg);
  }

  public void setHandheldTagObservation(int tagId, double txncDeg, double distanceToRobotMeters) {
    getOrEnableDemoContainer().setSimHandheldTagObservation(tagId, txncDeg, distanceToRobotMeters);
  }

  public void recordHandheldTagState() {
    DemoContainer demoContainer = demoContainerSupplier.get();
    if (demoContainer == null) {
      Logger.recordOutput("GymSim/DemoHandheld/TargetValid", false);
      Logger.recordOutput("GymSim/DemoHandheld/TargetValidNumeric", 0.0);
      Logger.recordOutput("GymSim/DemoHandheld/PoseFusionAllowed", true);
      Logger.recordOutput("GymSim/DemoHandheld/PoseFusionBlockedNumeric", 0.0);
      return;
    }
    demoContainer.recordHandheldTagStateForSim();
  }

  private DemoContainer getOrEnableDemoContainer() {
    DemoContainer demoContainer = demoContainerSupplier.get();
    if (demoContainer == null) {
      enableDemoMode();
      demoContainer = demoContainerSupplier.get();
    }
    return demoContainer;
  }
}
