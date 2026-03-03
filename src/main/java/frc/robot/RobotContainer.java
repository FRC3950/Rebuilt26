package frc.robot;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.SubsystemConstants.*;
import static frc.robot.Constants.SubsystemConstants.Turret.*;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;

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
import frc.robot.commands.TeleopDrive;
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
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private static final double CLIMBER_MANUAL_OUTPUT = 0.25;
  private static final double TURRET_STICK_DEADBAND = 0.20;

  // Subsystems
  private final Drive drive;
  private final Turret turret1;
  private final Turret turret2;
  private final Vision vision;
  private final Intake intake;
  private final Indexer indexer;
  private final Climber climber;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<HubShiftUtil.AutoWinnerMode> autoWinnerChooser;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
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
        // Sim robot, instantiate physics sim IO implementations
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
        // Replayed robot, disable IO implementations
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
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoWinnerChooser = new LoggedDashboardChooser<>("Auto Winner Override");
    autoWinnerChooser.addDefaultOption("Auto", HubShiftUtil.AutoWinnerMode.AUTO);
    autoWinnerChooser.addOption("Red", HubShiftUtil.AutoWinnerMode.RED);
    autoWinnerChooser.addOption("Blue", HubShiftUtil.AutoWinnerMode.BLUE);
    HubShiftUtil.setAutoWinnerModeSupplier(() -> autoWinnerChooser.get());

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Primary Driver Layout: https://tinyurl.com/5n7kv2up
    SmartDashboard.setDefaultBoolean("Snake Mode", true);

    Translation2d turretMidpointRobot = robotToTurret1.plus(robotToTurret2).times(0.5);
    Supplier<Translation2d> turretMidpointFieldSupplier =
        () ->
            drive
                .getPose()
                .getTranslation()
                .plus(turretMidpointRobot.rotateBy(drive.getPose().getRotation()));

    Trigger trenchDangerTrigger =
        Zones.TRENCH_ZONES
            .willContain(
                turretMidpointFieldSupplier, drive::getFieldRelativeSpeeds, TRENCH_ALIGN_TIME_SEC)
            .debounce(TRENCH_DEBOUNCE_SEC);

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
    new Trigger(DriverStation::isAutonomousEnabled)
        .onTrue(
            Commands.runOnce(
                () -> {
                  turret1.setHoodSafetyForcedDown(false);
                  turret2.setHoodSafetyForcedDown(false);
                }));

    Supplier<TargetingMode> autoTargetingModeSupplier =
        () -> {
          if (DriverStation.isAutonomousEnabled()) {
            return TargetingMode.HUB_AUTO;
          }
          return TurretTargeting.selectTargetingMode(
              true,
              trenchDangerTrigger.getAsBoolean(),
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
            trenchDangerTrigger::getAsBoolean,
            () -> SmartDashboard.getBoolean("Snake Mode", true),
            intake::isIntaking));

    Turret leftOperatorTurret = turret1;
    Turret rightOperatorTurret = turret2;
    if (robotToTurret2.getY() > robotToTurret1.getY()) {
      leftOperatorTurret = turret2;
      rightOperatorTurret = turret1;
    }
    // Created as effectively final so we can pass it into the manual targeting.
    final Turret finalLeftOperatorTurret = leftOperatorTurret;
    final Turret finalRightOperatorTurret = rightOperatorTurret;

    // Manual Ferry Mode - Left
    driver
        .povLeft()
        .and(() -> drive.getPose().getX() > neutralZoneMinX)
        .whileTrue(
            new FerryMode(turret1, drive, robotToTurret1, leftFerryTarget)
                .alongWith(new FerryMode(turret2, drive, robotToTurret2, leftFerryTarget)));

    // Manual Ferry Mode - Right
    driver
        .povRight()
        .and(() -> drive.getPose().getY() > neutralZoneMinX)
        .whileTrue(
            new FerryMode(turret1, drive, robotToTurret1, rightFerryTarget)
                .alongWith(new FerryMode(turret2, drive, robotToTurret2, rightFerryTarget)));

    // Intake
    driver
        .leftTrigger()
        .whileTrue(new IntakeCommand(intake, indexer, driver.rightTrigger()::getAsBoolean));

    driver.rightBumper().onTrue(intake.retractCommand());

    // Shoot/Feed Fuel
    driver
        .rightTrigger()
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

    // Climber
    driver.povDown().debounce(0.5).onTrue(climber.ClimbToggleCommand());

    // Operator Controls
    // Run Indexer
    operator
        .rightTrigger()
        .whileTrue(Commands.startEnd(indexer::startIndexer, indexer::stopIndexer, indexer));
    // Run HotDog
    operator.x().whileTrue(Commands.startEnd(indexer::startHotdog, indexer::stopHotdog, indexer));
    // Reset field-centric heading (Does nothing if robot can see tags to update its pose)
    operator
        .y()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));
    // Runs Intake
    operator.a().whileTrue(Commands.startEnd(intake::startIntake, intake::stopIntake, intake));
    // BackTake Intake
    operator
        .b()
        .and(operator.start().negate())
        .whileTrue(
            Commands.startEnd(
                () -> intake.setIntakeSpeed(-Constants.SubsystemConstants.Intake.intakeSpeed),
                intake::stopIntake,
                intake));
    // Backtake Indexer and HotDog
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
    // Retract Intake
    operator.leftBumper().onTrue(intake.retractCommand());
    // Extend Intake
    operator.rightBumper().onTrue(Commands.runOnce(intake::extend, intake));
    // Climber move up
    operator.povUp().whileTrue(climber.manualUpCommand(CLIMBER_MANUAL_OUTPUT));
    // Climber move down
    operator.povDown().whileTrue(climber.manualDownCommand(CLIMBER_MANUAL_OUTPUT));
    // Manual Turret Left
    operator
        .start()
        .and(
            () ->
                Turret.isManualStickActive(
                    operator.getLeftX(), operator.getLeftY(), TURRET_STICK_DEADBAND))
        .whileTrue(
            Commands.run(
                () ->
                    finalLeftOperatorTurret.runManualFieldHeading(
                        Turret.getFieldHeadingFromStick(operator.getLeftX(), operator.getLeftY()),
                        drive.getPose().getRotation()),
                finalLeftOperatorTurret));
    // Manual Turret Right
    operator
        .start()
        .and(
            () ->
                Turret.isManualStickActive(
                    operator.getRightX(), operator.getRightY(), TURRET_STICK_DEADBAND))
        .whileTrue(
            Commands.run(
                () ->
                    finalRightOperatorTurret.runManualFieldHeading(
                        Turret.getFieldHeadingFromStick(operator.getRightX(), operator.getRightY()),
                        drive.getPose().getRotation()),
                finalRightOperatorTurret));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
