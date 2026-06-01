package frc.robot.controls;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class DemoContainer {
  public enum DemoBindingMode {
    COMPETITION,
    CRAZY
  }

  private static final String MAX_SPEED_KEY = "Demo Mode/Max Speed MPS";
  private static final String REDUCED_SPEED_KEY = "Demo Mode/Reduced Speed While Shooting MPS";
  private static final double DEFAULT_MAX_SPEED_MPS = 2.0;

  private final Drive drive;
  private final EventLoop competitionButtonLoop = new EventLoop();
  private final EventLoop crazyButtonLoop = new EventLoop();
  private final LoggedDashboardChooser<DemoBindingMode> bindingModeChooser;

  private DemoBindingMode appliedBindingMode = DemoBindingMode.CRAZY;
  private DemoBindingMode lastPublishedSelectedBindingMode = null;

  public DemoContainer(
      CommandXboxController driver,
      CommandXboxController operator,
      Drive drive,
      Intake intake,
      Indexer indexer,
      Turret turret1,
      Turret turret2) {
    this.drive = drive;

    SmartDashboard.putNumber(MAX_SPEED_KEY, DEFAULT_MAX_SPEED_MPS);
    SmartDashboard.putNumber(REDUCED_SPEED_KEY, Constants.SubsystemConstants.Drive.reducedSpeed);

    bindingModeChooser = new LoggedDashboardChooser<>("Demo Mode/Bindings");
    bindingModeChooser.addDefaultOption("Crazy", DemoBindingMode.CRAZY);
    bindingModeChooser.addOption("Competition", DemoBindingMode.COMPETITION);

    CompBindings.configure(
        competitionButtonLoop,
        driver,
        operator,
        drive,
        intake,
        indexer,
        turret1,
        turret2,
        this::getMaxDriveSpeedMetersPerSec);
    CrazyBindings.configure(
        crazyButtonLoop,
        driver,
        drive,
        intake,
        indexer,
        turret1,
        turret2,
        this::getMaxDriveSpeedMetersPerSec);
  }

  public void checkMode(boolean isDisabled) {
    DemoBindingMode selectedBindingMode = getSelectedBindingMode();
    publishSelectedBindingMode(selectedBindingMode);

    if (!isDisabled || selectedBindingMode == appliedBindingMode) {
      return;
    }

    applySelectedBindingMode(selectedBindingMode);
  }

  public void applyCurrentBindingMode() {
    applySelectedBindingMode(getSelectedBindingMode());
  }

  public double getMaxDriveSpeedMetersPerSec() {
    return sanitizeDashboardSpeed(
        SmartDashboard.getNumber(MAX_SPEED_KEY, DEFAULT_MAX_SPEED_MPS),
        DEFAULT_MAX_SPEED_MPS,
        drive.getPhysicalMaxLinearSpeedMetersPerSec());
  }

  public double getReducedSpeedMetersPerSec() {
    return sanitizeDashboardSpeed(
        SmartDashboard.getNumber(
            REDUCED_SPEED_KEY, Constants.SubsystemConstants.Drive.reducedSpeed),
        Constants.SubsystemConstants.Drive.reducedSpeed,
        getMaxDriveSpeedMetersPerSec());
  }

  public static double sanitizeDashboardSpeed(
      double dashboardSpeedMetersPerSec,
      double defaultSpeedMetersPerSec,
      double maxSpeedMetersPerSec) {
    if (!Double.isFinite(dashboardSpeedMetersPerSec) || dashboardSpeedMetersPerSec <= 0.0) {
      return Math.min(defaultSpeedMetersPerSec, maxSpeedMetersPerSec);
    }
    return Math.min(dashboardSpeedMetersPerSec, maxSpeedMetersPerSec);
  }

  private void applySelectedBindingMode(DemoBindingMode bindingMode) {
    CommandScheduler.getInstance()
        .setActiveButtonLoop(
            switch (bindingMode) {
              case COMPETITION -> competitionButtonLoop;
              case CRAZY -> crazyButtonLoop;
            });

    appliedBindingMode = bindingMode;
    SmartDashboard.putString("Demo Mode/Bindings Applied", appliedBindingMode.name());
    Logger.recordOutput("Controls/DemoBindingModeApplied", appliedBindingMode.name());
  }

  private void publishSelectedBindingMode(DemoBindingMode selectedBindingMode) {
    if (selectedBindingMode == lastPublishedSelectedBindingMode) {
      return;
    }

    lastPublishedSelectedBindingMode = selectedBindingMode;
    SmartDashboard.putString("Demo Mode/Bindings Selected", selectedBindingMode.name());
    Logger.recordOutput("Controls/DemoBindingModeSelected", selectedBindingMode.name());
  }

  private DemoBindingMode getSelectedBindingMode() {
    DemoBindingMode selectedBindingMode = bindingModeChooser.get();
    return selectedBindingMode != null ? selectedBindingMode : DemoBindingMode.CRAZY;
  }
}
