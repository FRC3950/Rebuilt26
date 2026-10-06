package frc.robot.controls;

import static frc.robot.Constants.FieldConstants.getCloserFerryTarget;
import static frc.robot.Constants.FieldConstants.getHubTranslation;
import static frc.robot.Constants.FieldConstants.isRobotInNeutralZone;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret1;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretTargeting;

public final class CrazyModeBindings {
  private CrazyModeBindings() {}

  public static void configure(
      CommandXboxController driver,
      Drive drive,
      Intake intake,
      Indexer indexer,
      Turret turret1,
      Turret turret2) {
    driver
        .leftTrigger(0.5)
        .whileTrue(
            new IntakeCommand(
                intake,
                () -> {
                  var speeds = drive.getRobotRelativeSpeeds();
                  return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
                }));

    Trigger neutralZoneFerryTrigger =
        new Trigger(() -> isRobotInNeutralZone(drive.getPose().getX()));
    neutralZoneFerryTrigger.whileTrue(
        Commands.parallel(
            new TurretTargeting(
                turret1,
                drive,
                robotToTurret1,
                () -> getCloserFerryTarget(drive.getPose().getTranslation())),
            new TurretTargeting(
                turret2,
                drive,
                robotToTurret2,
                () -> getCloserFerryTarget(drive.getPose().getTranslation()))));

    driver.rightBumper().onTrue(intake.retractCommand());

    driver
        .rightTrigger(0.5)
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

    driver
        .y()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));
    driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> {
                  Translation2d robotToHub =
                      getHubTranslation().minus(drive.getPose().getTranslation());
                  return new Rotation2d(robotToHub.getX(), robotToHub.getY())
                      .rotateBy(new Rotation2d(Math.PI));
                }));
    driver
        .b()
        .and(driver.start().negate())
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
        .start()
        .and(driver.b())
        .whileTrue(Commands.startEnd(intake::reverseIntake, intake::stopIntake, intake));
    new Trigger(() -> driver.getHID().getPOV() == 180)
        .onTrue(Commands.runOnce(Turret::toggleTurretMode))
        .debounce(0.25);
  }
}
