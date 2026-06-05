package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class MirroringAutoChooser {
  private static final String NONE_NAME = "None";
  private static final String MIRROR_AUTO_KEY = "Auto/Mirror Selected Auto";
  private static final String MIRROR_APPLIED_KEY = "Auto/Mirror Applied";
  private static final String SELECTED_AUTO_KEY = "Auto/Selected Name";

  private final LoggedDashboardChooser<String> autoChooser;

  public MirroringAutoChooser(String chooserKey) {
    SmartDashboard.setDefaultBoolean(MIRROR_AUTO_KEY, false);

    autoChooser = new LoggedDashboardChooser<>(chooserKey);
    autoChooser.addDefaultOption(NONE_NAME, "");
    for (String autoName : AutoBuilder.getAllAutoNames()) {
      autoChooser.addOption(autoName, autoName);
    }
  }

  public Command getAutonomousCommand() {
    String selectedAutoName = autoChooser.get();
    boolean mirror = SmartDashboard.getBoolean(MIRROR_AUTO_KEY, false);
    SmartDashboard.putBoolean(MIRROR_APPLIED_KEY, mirror);
    SmartDashboard.putString(SELECTED_AUTO_KEY, selectedAutoName == null ? "" : selectedAutoName);
    return buildAutonomousCommand(selectedAutoName, mirror, PathPlannerAuto::new);
  }

  static Command buildAutonomousCommand(
      String selectedAutoName, boolean mirror, AutoCommandFactory autoCommandFactory) {
    if (selectedAutoName == null || selectedAutoName.isBlank()) {
      return Commands.none();
    }
    return autoCommandFactory.build(selectedAutoName, mirror);
  }

  @FunctionalInterface
  interface AutoCommandFactory {
    Command build(String autoName, boolean mirror);
  }
}
