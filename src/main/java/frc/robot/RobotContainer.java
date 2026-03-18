package frc.robot;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.SubsystemConstants.CANivore;
import static frc.robot.Constants.SubsystemConstants.Turret.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretTargeting;
import frc.robot.subsystems.turret.TurretTargeting.TargetingMode;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.Zones;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private final Drive drive;
  private final Turret turret1;
  private final Turret turret2;
  private final Vision vision;
  private final Intake intake;
  private final Indexer indexer;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    turret1 =
        new Turret(
            azimuthID,
            azimuthConfig,
            HOOD_SERVO_CHANNEL_1,
            flywheelID,
            flywheelConfig,
            flywheelFollowerID,
            CANivore);
    turret2 =
        new Turret(
            azimuthID2,
            azimuthConfig,
            HOOD_SERVO_CHANNEL_2,
            flywheelID2,
            flywheelConfig,
            flywheelFollowerID2,
            CANivore);

    intake = new Intake();
    indexer = new Indexer();

    NamedCommands.registerCommand(
        "EnableHoodSafetyMode",
        Commands.runOnce(
            () -> {
              turret1.setHoodSafetyForcedDown(true);
              turret2.setHoodSafetyForcedDown(true);
            }));
    NamedCommands.registerCommand(
        "DisableHoodSafetyMode",
        Commands.runOnce(
            () -> {
              turret1.setHoodSafetyForcedDown(false);
              turret2.setHoodSafetyForcedDown(false);
            }));
    NamedCommands.registerCommand("Extend Intake", Commands.runOnce(intake::extend, intake));
    NamedCommands.registerCommand("Retract Intake", Commands.runOnce(intake::retract, intake));
    NamedCommands.registerCommand("Start Intake", Commands.runOnce(intake::startIntake, intake));
    NamedCommands.registerCommand("Stop Intake", Commands.runOnce(intake::stopIntake, intake));

    NamedCommands.registerCommand("Start Hotdog", Commands.runOnce(indexer::startHotdog, indexer));
    NamedCommands.registerCommand("Stop Hotdog", Commands.runOnce(indexer::stopHotdog, indexer));

    NamedCommands.registerCommand(
        "Start Shoot",
        Commands.runOnce(
            () -> {
              indexer.startIndexer();
              indexer.startHotdog();
            },
            indexer));
    NamedCommands.registerCommand(
        "End Shoot",
        Commands.runOnce(
            () -> {
              indexer.stopIndexer();
              indexer.stopHotdog();
            },
            indexer));

    SmartDashboard.putData("Turret Subsystem", turret1);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices: ", AutoBuilder.buildAutoChooser());
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdTranslationQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdTranslationQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        drive.sysIdTranslationDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        drive.sysIdTranslationDynamic(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();
    applyCompetitionDefaults();
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void configureButtonBindings() {
    BooleanSupplier trenchDangerSupplier = this::isTrenchDanger;

    Trigger trenchDangerTrigger = new Trigger(trenchDangerSupplier).debounce(TRENCH_DEBOUNCE_SEC);
    Trigger trenchSafetyEnabledTrigger = new Trigger(() -> !DriverStation.isAutonomousEnabled());
    Trigger activeTrenchSafetyTrigger = trenchSafetyEnabledTrigger.and(trenchDangerTrigger);

    activeTrenchSafetyTrigger.onTrue(
        Commands.runOnce(
            () -> {
              turret1.setHoodSafetyForcedDown(true);
              turret2.setHoodSafetyForcedDown(true);
            }));
    activeTrenchSafetyTrigger.onFalse(
        Commands.runOnce(
            () -> {
              turret1.setHoodSafetyForcedDown(false);
              turret2.setHoodSafetyForcedDown(false);
            }));

    driver.leftTrigger(0.5).whileTrue(new IntakeCommand(intake));

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

    operator
        .rightTrigger(0.5)
        .whileTrue(Commands.startEnd(indexer::startIndexer, indexer::stopIndexer, indexer));
    operator.x().whileTrue(Commands.startEnd(indexer::startHotdog, indexer::stopHotdog, indexer));
    operator
        .y()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));
    operator.a().whileTrue(Commands.startEnd(intake::startIntake, intake::stopIntake, intake));
    operator
        .b()
        .and(operator.start().negate())
        .whileTrue(
            Commands.startEnd(
                () -> intake.setIntakeSpeed(-Constants.SubsystemConstants.Intake.intakeSpeed),
                intake::stopIntake,
                intake));
    operator
        .start()
        .and(operator.b())
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
    operator.leftBumper().onTrue(intake.retractCommand());
    operator.rightBumper().onTrue(Commands.runOnce(intake::extend, intake));
  }

  private void applyCompetitionDefaults() {
    BooleanSupplier trenchDangerSupplier = this::isTrenchDanger;
    Supplier<TargetingMode> autoTargetingModeSupplier =
        () -> {
          if (DriverStation.isAutonomousEnabled()) {
            return TargetingMode.HUB_AUTO;
          }
          return TurretTargeting.selectTargetingMode(
              true,
              trenchDangerSupplier.getAsBoolean(),
              Zones.NEUTRAL_ZONE.contains(drive.getPose().getTranslation()));
        };

    turret1.setDefaultCommand(
        new TurretTargeting(turret1, drive, robotToTurret1, autoTargetingModeSupplier));
    turret2.setDefaultCommand(
        new TurretTargeting(turret2, drive, robotToTurret2, autoTargetingModeSupplier));
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
  }

  private boolean isTrenchDanger() {
    return Zones.TRENCH_ZONES.willContain(
        getTurretMidpointField(), drive.getFieldRelativeSpeeds(), TRENCH_ALIGN_TIME_SEC);
  }

  private Translation2d getTurretMidpointField() {
    Translation2d turretMidpointRobot = robotToTurret1.plus(robotToTurret2).times(0.5);
    return drive
        .getPose()
        .getTranslation()
        .plus(turretMidpointRobot.rotateBy(drive.getPose().getRotation()));
  }
}
