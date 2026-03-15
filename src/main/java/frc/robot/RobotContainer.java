package frc.robot;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.SubsystemConstants.CANivore;
import static frc.robot.Constants.SubsystemConstants.Turret.*;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.controls.TuneModeBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.FerryMode;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretTargeting;
import frc.robot.subsystems.turret.TurretTargeting.TargetingMode;
import frc.robot.subsystems.turret.turret_base.Azimuth.LimitSwitchChannel;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.SmartShotRelease;
import frc.robot.util.Zones;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  public enum CodeMode {
    COMPETITION,
    TUNE
  }

  private static final double CLIMBER_MANUAL_OUTPUT = 0.25;
  private static final double TURRET_STICK_DEADBAND = 0.20;

  private final Drive drive;
  private final Turret turret1;
  private final Turret turret2;
  private final Turret leftSideTurret;
  private final Turret rightSideTurret;
  private final Vision vision;
  private final Intake intake;
  private final Indexer indexer;
  private final Climber climber;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final EventLoop competitionButtonLoop = new EventLoop();
  private final EventLoop tuneButtonLoop = new EventLoop();

  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<HubShiftUtil.AutoWinnerMode> autoWinnerChooser;
  private final LoggedDashboardChooser<CodeMode> codeModeChooser;

  private CodeMode appliedCodeMode;

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
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));
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
                    camera0Name, VisionConstants.robotToCamera0, drive::getPose));
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
            LimitSwitchChannel.S1,
            HOOD_SERVO_CHANNEL_1,
            flywheelID,
            flywheelConfig,
            flywheelFollowerID,
            CANivore);
    turret2 =
        new Turret(
            azimuthID2,
            azimuthConfig,
            LimitSwitchChannel.S2,
            HOOD_SERVO_CHANNEL_2,
            flywheelID2,
            flywheelConfig,
            flywheelFollowerID2,
            CANivore);

    Turret computedLeftSideTurret = turret1;
    Turret computedRightSideTurret = turret2;
    if (robotToTurret2.getY() > robotToTurret1.getY()) {
      computedLeftSideTurret = turret2;
      computedRightSideTurret = turret1;
    }
    leftSideTurret = computedLeftSideTurret;
    rightSideTurret = computedRightSideTurret;

    intake = new Intake();
    indexer = new Indexer();
    climber = new Climber(drive::getPose);

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

    autoWinnerChooser = new LoggedDashboardChooser<>("Auto Winner Override");
    autoWinnerChooser.addDefaultOption("Auto", HubShiftUtil.AutoWinnerMode.AUTO);
    autoWinnerChooser.addOption("Red", HubShiftUtil.AutoWinnerMode.RED);
    autoWinnerChooser.addOption("Blue", HubShiftUtil.AutoWinnerMode.BLUE);
    HubShiftUtil.setAutoWinnerModeSupplier(autoWinnerChooser::get);

    codeModeChooser = new LoggedDashboardChooser<>("Code Mode");
    codeModeChooser.addDefaultOption("Competition", CodeMode.COMPETITION);
    codeModeChooser.addOption("TUNE", CodeMode.TUNE);

    configureButtonBindings();
    configureTuneModeBindings();
    SmartDashboard.setDefaultBoolean("Snake Mode", true);

    applyCodeMode(getSelectedCodeMode());
  }

  public void periodic() {
    CodeMode selectedCodeMode = getSelectedCodeMode();
    SmartDashboard.putString("Code Mode/Selected", selectedCodeMode.name());
    Logger.recordOutput("Controls/CodeModeSelected", selectedCodeMode.name());

    if (!shouldApplyCodeMode(selectedCodeMode, appliedCodeMode, DriverStation.isDisabled())) {
      return;
    }

    applyCodeMode(selectedCodeMode);
  }

  static boolean shouldApplyCodeMode(
      CodeMode selectedCodeMode, CodeMode currentCodeMode, boolean isDisabled) {
    return isDisabled && selectedCodeMode != currentCodeMode;
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void configureButtonBindings() {
    BooleanSupplier trenchDangerSupplier = this::isTrenchDanger;

    Trigger trenchDangerTrigger =
        new Trigger(competitionButtonLoop, trenchDangerSupplier).debounce(TRENCH_DEBOUNCE_SEC);
    Trigger trenchSafetyEnabledTrigger =
        new Trigger(competitionButtonLoop, () -> !DriverStation.isAutonomousEnabled());
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
    new Trigger(competitionButtonLoop, DriverStation::isAutonomousEnabled)
        .onTrue(
            Commands.runOnce(
                () -> {
                  turret1.setHoodSafetyForcedDown(false);
                  turret2.setHoodSafetyForcedDown(false);
                }));

    new Trigger(competitionButtonLoop, () -> driver.getHID().getPOV() == 270)
        .and(() -> drive.getPose().getX() > neutralZoneMinX)
        .whileTrue(
            new FerryMode(turret1, drive, robotToTurret1, leftFerryTarget)
                .alongWith(new FerryMode(turret2, drive, robotToTurret2, leftFerryTarget)));

    new Trigger(competitionButtonLoop, () -> driver.getHID().getPOV() == 90)
        .and(() -> drive.getPose().getY() > neutralZoneMinX)
        .whileTrue(
            new FerryMode(turret1, drive, robotToTurret1, rightFerryTarget)
                .alongWith(new FerryMode(turret2, drive, robotToTurret2, rightFerryTarget)));

    driver
        .leftTrigger(0.5, competitionButtonLoop)
        .whileTrue(
            new IntakeCommand(
                intake, indexer, driver.rightTrigger(0.5, competitionButtonLoop)::getAsBoolean));

    driver.rightBumper(competitionButtonLoop).onTrue(intake.retractCommand());

    driver
        .rightTrigger(0.5, competitionButtonLoop)
        .and(
            () ->
                SmartShotRelease.canShoot(
                    drive.getPose().getTranslation().getDistance(hubTranslation)))
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

    new Trigger(competitionButtonLoop, () -> driver.getHID().getPOV() == 180)
        .debounce(0.5)
        .onTrue(climber.ClimbToggleCommand());

    operator
        .rightTrigger(0.5, competitionButtonLoop)
        .whileTrue(Commands.startEnd(indexer::startIndexer, indexer::stopIndexer, indexer));
    operator
        .x(competitionButtonLoop)
        .whileTrue(Commands.startEnd(indexer::startHotdog, indexer::stopHotdog, indexer));
    operator
        .y(competitionButtonLoop)
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));
    operator
        .a(competitionButtonLoop)
        .whileTrue(Commands.startEnd(intake::startIntake, intake::stopIntake, intake));
    operator
        .b(competitionButtonLoop)
        .and(operator.start(competitionButtonLoop).negate())
        .whileTrue(
            Commands.startEnd(
                () -> intake.setIntakeSpeed(-Constants.SubsystemConstants.Intake.intakeSpeed),
                intake::stopIntake,
                intake));
    operator
        .start(competitionButtonLoop)
        .and(operator.b(competitionButtonLoop))
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
    operator.leftBumper(competitionButtonLoop).onTrue(intake.retractCommand());
    operator.rightBumper(competitionButtonLoop).onTrue(Commands.runOnce(intake::extend, intake));
    new Trigger(competitionButtonLoop, () -> operator.getHID().getPOV() == 0)
        .whileTrue(climber.manualUpCommand(CLIMBER_MANUAL_OUTPUT));
    new Trigger(competitionButtonLoop, () -> operator.getHID().getPOV() == 180)
        .whileTrue(climber.manualDownCommand(CLIMBER_MANUAL_OUTPUT));
    operator
        .start(competitionButtonLoop)
        .and(
            () ->
                Turret.isManualStickActive(
                    operator.getLeftX(), operator.getLeftY(), TURRET_STICK_DEADBAND))
        .whileTrue(
            Commands.run(
                () ->
                    leftSideTurret.runManualFieldHeading(
                        Turret.getFieldHeadingFromStick(operator.getLeftX(), operator.getLeftY()),
                        drive.getPose().getRotation()),
                leftSideTurret));
    operator
        .start(competitionButtonLoop)
        .and(
            () ->
                Turret.isManualStickActive(
                    operator.getRightX(), operator.getRightY(), TURRET_STICK_DEADBAND))
        .whileTrue(
            Commands.run(
                () ->
                    rightSideTurret.runManualFieldHeading(
                        Turret.getFieldHeadingFromStick(operator.getRightX(), operator.getRightY()),
                        drive.getPose().getRotation()),
                rightSideTurret));
  }

  private void configureTuneModeBindings() {
    TuneModeBindings.configure(
        tuneButtonLoop, driver, drive, intake, indexer, leftSideTurret, rightSideTurret);
  }

  private void applyCodeMode(CodeMode codeMode) {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.cancelAll();
    scheduler.removeDefaultCommand(drive);
    scheduler.removeDefaultCommand(turret1);
    scheduler.removeDefaultCommand(turret2);

    switch (codeMode) {
      case COMPETITION:
        scheduler.setActiveButtonLoop(competitionButtonLoop);
        applyCompetitionDefaults();
        break;
      case TUNE:
        scheduler.setActiveButtonLoop(tuneButtonLoop);
        applyTuneDefaults();
        break;
    }

    appliedCodeMode = codeMode;
    SmartDashboard.putString("Code Mode/Applied", appliedCodeMode.name());
    Logger.recordOutput("Controls/CodeModeApplied", appliedCodeMode.name());
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
        new TeleopDrive(
            drive,
            driver,
            trenchDangerSupplier,
            () -> SmartDashboard.getBoolean("Snake Mode", true),
            intake::isIntaking));
  }

  private void applyTuneDefaults() {
    drive.setDefaultCommand(TuneModeBindings.createDriveDefaultCommand(drive, driver));
    turret1.setDefaultCommand(
        Commands.run(turret1::holdCurrentPositionWithStoppedFlywheel, turret1));
    turret2.setDefaultCommand(
        Commands.run(turret2::holdCurrentPositionWithStoppedFlywheel, turret2));
  }

  private CodeMode getSelectedCodeMode() {
    CodeMode selectedCodeMode = codeModeChooser.get();
    return selectedCodeMode != null ? selectedCodeMode : CodeMode.COMPETITION;
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
