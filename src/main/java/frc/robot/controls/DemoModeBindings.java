package frc.robot.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

/**
 * Driver bindings for the Demo code mode. Turret aiming is handled by DemoShooter, so there is no
 * ferrying, no face-the-hub, and nothing that depends on field odometry here.
 */
public final class DemoModeBindings {
  private DemoModeBindings() {}

  public static void configure(
      EventLoop buttonLoop,
      CommandXboxController driver,
      Drive drive,
      Intake intake,
      Indexer indexer) {
    driver
        .leftTrigger(0.5, buttonLoop)
        .whileTrue(
            new IntakeCommand(
                intake,
                () -> {
                  var speeds = drive.getRobotRelativeSpeeds();
                  return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
                }));

    driver.rightBumper(buttonLoop).onTrue(intake.retractCommand());

    driver
        .rightTrigger(0.5, buttonLoop)
        .whileTrue(
            Commands.startEnd(
                () -> {
                  indexer.startIndexer();
                  indexer.startHotdog();
                },
                () -> {
                  indexer.stopIndexer();
                  indexer.stopHotdog();
                },
                indexer));

    // Re-zero heading for field-relative driving (robot's current facing = away from driver).
    driver
        .y(buttonLoop)
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));

    driver
        .b(buttonLoop)
        .and(driver.start(buttonLoop).negate())
        .whileTrue(
            Commands.startEnd(
                () -> {
                  indexer.setIndexerSpeed(-Constants.SubsystemConstants.Indexer.indexerSpeed);
                  indexer.setHotdogSpeed(-Constants.SubsystemConstants.Indexer.hotdogSpeed);
                },
                () -> {
                  indexer.stopIndexer();
                  indexer.stopHotdog();
                },
                indexer));
    driver
        .start(buttonLoop)
        .and(driver.b(buttonLoop))
        .whileTrue(Commands.startEnd(intake::reverseIntake, intake::stopIntake, intake));
  }
}
