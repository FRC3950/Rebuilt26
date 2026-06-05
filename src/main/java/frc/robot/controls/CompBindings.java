package frc.robot.controls;

import static frc.robot.Constants.FieldConstants.getCloserFerryTarget;
import static frc.robot.Constants.FieldConstants.getHubTranslation;
import static frc.robot.Constants.FieldConstants.isRobotInNeutralZone;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret1;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretTargeting;
import java.util.function.DoubleSupplier;

public final class CompBindings {
  private CompBindings() {}

  public static void configure(
      EventLoop buttonLoop,
      CommandXboxController driver,
      CommandXboxController operator,
      Drive drive,
      Intake intake,
      Indexer indexer,
      Turret turret1,
      Turret turret2) {
    configure(
        buttonLoop,
        driver,
        operator,
        drive,
        intake,
        indexer,
        turret1,
        turret2,
        drive::getMaxLinearSpeedMetersPerSec);
  }

  public static void configure(
      EventLoop buttonLoop,
      CommandXboxController driver,
      CommandXboxController operator,
      Drive drive,
      Intake intake,
      Indexer indexer,
      Turret turret1,
      Turret turret2,
      DoubleSupplier maxLinearSpeedSupplier) {
    operator
        .leftTrigger(0.5, buttonLoop)
        .whileTrue(
            new IntakeCommand(
                intake,
                () -> {
                  var speeds = drive.getRobotRelativeSpeeds();
                  return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
                }));
    Trigger neutralZoneFerryTrigger =
        new Trigger(buttonLoop, () -> isRobotInNeutralZone(drive.getPose().getX()));
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

    operator.rightBumper(buttonLoop).onTrue(intake.retractCommand());

    operator
        .rightTrigger(0.5, buttonLoop)
        .whileTrue(
            Commands.startEnd(indexer::requestForwardFeed, indexer::stopForwardFeed, indexer));

    driver
        .y(buttonLoop)
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));
    driver.x(buttonLoop).whileTrue(DriveCommands.xWheelLock(drive));
    driver
        .a(buttonLoop)
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
                },
                maxLinearSpeedSupplier));
    operator
        .b(buttonLoop)
        .whileTrue(
            Commands.startEnd(
                () -> {
                  intake.reverseIntake();
                  indexer.reverseHotdog();
                },
                () -> {
                  intake.stopIntake();
                  indexer.stopHotdog();
                },
                intake,
                indexer));

    operator.a(buttonLoop).onTrue(Commands.runOnce(Turret::toggleTurretMode)).debounce(0.25);
  }
}
