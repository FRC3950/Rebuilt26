package frc.robot;

import static frc.robot.Constants.FieldConstants.getCloserFerryTarget;
import static frc.robot.Constants.FieldConstants.getHubTranslation;
import static frc.robot.Constants.FieldConstants.isRobotInNeutralZone;
import static frc.robot.Constants.SubsystemConstants.CANivore;
import static frc.robot.Constants.SubsystemConstants.Turret.HOOD_SERVO_CHANNEL_1;
import static frc.robot.Constants.SubsystemConstants.Turret.HOOD_SERVO_CHANNEL_2;
import static frc.robot.Constants.SubsystemConstants.Turret.azimuthID;
import static frc.robot.Constants.SubsystemConstants.Turret.azimuthID2;
import static frc.robot.Constants.SubsystemConstants.Turret.flywheelConfig;
import static frc.robot.Constants.SubsystemConstants.Turret.flywheelFollowerID;
import static frc.robot.Constants.SubsystemConstants.Turret.flywheelFollowerID2;
import static frc.robot.Constants.SubsystemConstants.Turret.flywheelID;
import static frc.robot.Constants.SubsystemConstants.Turret.flywheelID2;
import static frc.robot.Constants.SubsystemConstants.Turret.leftAzimuthConfig;
import static frc.robot.Constants.SubsystemConstants.Turret.leftMaxAzimuthControlAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.leftMinAzimuthControlAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.rightAzimuthConfig;
import static frc.robot.Constants.SubsystemConstants.Turret.rightMaxAzimuthControlAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.rightMinAzimuthControlAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret1;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret2;

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
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommand;
import frc.robot.controls.CrazyModeBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.sim.FuelSimCommand;
import frc.robot.sim.FuelSimulationController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretTargeting;
import frc.robot.subsystems.turret.TurretVisualization;
import frc.robot.subsystems.turret.turret_base.azimuth.Azimuth;
import frc.robot.subsystems.turret.turret_base.azimuth.AzimuthIO;
import frc.robot.subsystems.turret.turret_base.azimuth.AzimuthIOTalonFX;
import frc.robot.subsystems.turret.turret_base.flywheels.Flywheels;
import frc.robot.subsystems.turret.turret_base.flywheels.FlywheelsIO;
import frc.robot.subsystems.turret.turret_base.flywheels.FlywheelsIOTalonFX;
import frc.robot.subsystems.turret.turret_base.hood.Hood;
import frc.robot.subsystems.turret.turret_base.hood.HoodIO;
import frc.robot.subsystems.turret.turret_base.hood.HoodIOServoHub;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.Field2dPublisher;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  public enum BindingMode {
    COMPETITION,
    CRAZY
  }

  private final Drive drive;
  private final Turret turret1;
  private final Turret turret2;
  private final TurretVisualization turretVisualization;
  private final Vision vision;
  private final Field2dPublisher fieldPublisher;
  private final Intake intake;
  private final Indexer indexer;
  private final Command simulationCommand;
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final EventLoop competitionButtonLoop = new EventLoop();
  private final EventLoop crazyButtonLoop = new EventLoop();

  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<BindingMode> bindingModeChooser;

  private BindingMode appliedBindingMode = BindingMode.COMPETITION;
  private BindingMode lastPublishedSelectedBindingMode = null;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        intake = new Intake(new IntakeIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        drive =
            new Drive(
                new GyroIOPigeon2(),
                intake::isIntaking,
                indexer::isFeedingForward,
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
        turret1 = createRealLeftTurret();
        turret2 = createRealRightTurret();
        break;

      case SIM:
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        drive =
            new Drive(
                new GyroIO() {},
                intake::isIntaking,
                indexer::isFeedingForward,
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
        turret1 = createNoOpLeftTurret();
        turret2 = createNoOpRightTurret();
        break;

      default:
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        drive =
            new Drive(
                new GyroIO() {},
                intake::isIntaking,
                indexer::isFeedingForward,
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        turret1 = createNoOpLeftTurret();
        turret2 = createNoOpRightTurret();
        break;
    }
    turretVisualization = new TurretVisualization(turret1, turret2);
    fieldPublisher = new Field2dPublisher("Field", drive::getPose);

    if (Constants.currentMode == Constants.Mode.SIM) {
      FuelSimulationController fuelSimulationController =
          new FuelSimulationController(
              drive::getPose,
              drive::getFieldRelativeSpeeds,
              intake::getCommandedRollerSpeed,
              intake::isPivotCommandedDown,
              indexer::isFeedingForward,
              new FuelSimulationController.TurretSimSource(
                  robotToTurret1,
                  turret1::getCommandedAzimuthDeg,
                  turret1::getCommandedHoodAngleDeg,
                  turret1::getCommandedFlywheelRps),
              new FuelSimulationController.TurretSimSource(
                  robotToTurret2,
                  turret2::getCommandedAzimuthDeg,
                  turret2::getCommandedHoodAngleDeg,
                  turret2::getCommandedFlywheelRps));
      simulationCommand = new FuelSimCommand(fuelSimulationController);
      SmartDashboard.putData(
          "Fuel Sim/Reset Field Fuel",
          Commands.runOnce(fuelSimulationController::resetFieldFuelToStartingConfiguration)
              .ignoringDisable(true));
    } else {
      simulationCommand = null;
    }

    NamedCommands.registerCommand("Extend Intake", intake.extendCommand());
    NamedCommands.registerCommand("Start Intake", intake.onIntake());
    NamedCommands.registerCommand("Stop Intake", intake.offIntake());
    NamedCommands.registerCommand(
        "Intake While Held", Commands.startEnd(intake::startIntake, intake::stopIntake, intake));
    NamedCommands.registerCommand("Start Hotdog", Commands.runOnce(indexer::startHotdog, indexer));
    NamedCommands.registerCommand("Stop Hotdog", Commands.runOnce(indexer::stopHotdog, indexer));

    NamedCommands.registerCommand(
        "Start Shoot",
        Commands.runOnce(
            () -> {
              indexer.startIndexer();
              indexer.startHotdog();
              intake.startIntake();
            },
            indexer,
            intake));
    NamedCommands.registerCommand(
        "End Shoot",
        Commands.runOnce(
            () -> {
              indexer.stopIndexer();
              indexer.stopHotdog();
              intake.stopIntake();
            },
            indexer,
            intake));

    SmartDashboard.putData("Turret Subsystem", turret1);
    autoChooser = new LoggedDashboardChooser<>("Auto Choices: ", AutoBuilder.buildAutoChooser());

    bindingModeChooser = new LoggedDashboardChooser<>("Code Mode");
    bindingModeChooser.addDefaultOption("Competition", BindingMode.COMPETITION);
    bindingModeChooser.addOption("CRAZY", BindingMode.CRAZY);

    configureCompetitionBindings();
    configureCrazyBindings();
    applyCompetitionDefaults();
    applyBindingMode(BindingMode.COMPETITION);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Command getSimulationCommand() {
    return simulationCommand;
  }

  public void stopAutonomousActions() {
    intake.stopIntake();
    indexer.stopIndexer();
    indexer.stopHotdog();
  }

  public void checkMode() {
    BindingMode selectedBindingMode = getSelectedBindingMode();
    publishSelectedBindingMode(selectedBindingMode);

    if (!shouldApplyBindingMode(
        selectedBindingMode, appliedBindingMode, DriverStation.isDisabled())) {
      return;
    }

    applyBindingMode(selectedBindingMode);
  }

  static boolean shouldApplyBindingMode(
      BindingMode selectedBindingMode, BindingMode currentBindingMode, boolean isDisabled) {
    return isDisabled && selectedBindingMode != currentBindingMode;
  }

  private void configureCompetitionBindings() {
    operator
        .leftTrigger(0.5, competitionButtonLoop)
        .whileTrue(
            new IntakeCommand(
                intake,
                () -> {
                  var speeds = drive.getRobotRelativeSpeeds();
                  return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
                }));
    Trigger neutralZoneFerryTrigger =
        new Trigger(competitionButtonLoop, () -> isRobotInNeutralZone(drive.getPose().getX()));
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

    operator.rightBumper(competitionButtonLoop).onTrue(intake.retractCommand());

    operator
        .rightTrigger(0.5, competitionButtonLoop)
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
        .y(competitionButtonLoop)
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));
    driver
        .a(competitionButtonLoop)
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
    operator
        .b(competitionButtonLoop)
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

    operator
        .a(competitionButtonLoop)
        .onTrue(Commands.runOnce(Turret::toggleTurretMode))
        .debounce(0.25);
  }

  private Turret createRealLeftTurret() {
    return new Turret(
        "LeftTurret",
        new Azimuth(
            "Turret/Left/Azimuth",
            new AzimuthIOTalonFX(azimuthID, leftAzimuthConfig, CANivore, true)),
        new Hood("Turret/Left/Hood", new HoodIOServoHub(HOOD_SERVO_CHANNEL_2)),
        new Flywheels(
            "Turret/Left/Flywheels",
            new FlywheelsIOTalonFX(flywheelID, flywheelConfig, flywheelFollowerID, CANivore)),
        leftMinAzimuthControlAngle,
        leftMaxAzimuthControlAngle);
  }

  private Turret createRealRightTurret() {
    return new Turret(
        "RightTurret",
        new Azimuth(
            "Turret/Right/Azimuth",
            new AzimuthIOTalonFX(azimuthID2, rightAzimuthConfig, CANivore, false)),
        new Hood("Turret/Right/Hood", new HoodIOServoHub(HOOD_SERVO_CHANNEL_1)),
        new Flywheels(
            "Turret/Right/Flywheels",
            new FlywheelsIOTalonFX(flywheelID2, flywheelConfig, flywheelFollowerID2, CANivore)),
        rightMinAzimuthControlAngle,
        rightMaxAzimuthControlAngle);
  }

  private Turret createNoOpLeftTurret() {
    return new Turret(
        "LeftTurret",
        new Azimuth("Turret/Left/Azimuth", new AzimuthIO() {}),
        new Hood("Turret/Left/Hood", new HoodIO() {}),
        new Flywheels("Turret/Left/Flywheels", new FlywheelsIO() {}),
        leftMinAzimuthControlAngle,
        leftMaxAzimuthControlAngle);
  }

  private Turret createNoOpRightTurret() {
    return new Turret(
        "RightTurret",
        new Azimuth("Turret/Right/Azimuth", new AzimuthIO() {}),
        new Hood("Turret/Right/Hood", new HoodIO() {}),
        new Flywheels("Turret/Right/Flywheels", new FlywheelsIO() {}),
        rightMinAzimuthControlAngle,
        rightMaxAzimuthControlAngle);
  }

  private void configureCrazyBindings() {
    CrazyModeBindings.configure(crazyButtonLoop, driver, drive, intake, indexer, turret1, turret2);
  }

  private void applyBindingMode(BindingMode bindingMode) {
    CommandScheduler.getInstance()
        .setActiveButtonLoop(
            switch (bindingMode) {
              case COMPETITION -> competitionButtonLoop;
              case CRAZY -> crazyButtonLoop;
            });

    appliedBindingMode = bindingMode;
    SmartDashboard.putString("Code Mode/Applied", appliedBindingMode.name());
    Logger.recordOutput("Controls/BindingModeApplied", appliedBindingMode.name());
  }

  private void publishSelectedBindingMode(BindingMode selectedBindingMode) {
    if (selectedBindingMode == lastPublishedSelectedBindingMode) {
      return;
    }

    lastPublishedSelectedBindingMode = selectedBindingMode;
    SmartDashboard.putString("Code Mode/Selected", selectedBindingMode.name());
    Logger.recordOutput("Controls/BindingModeSelected", selectedBindingMode.name());
  }

  private BindingMode getSelectedBindingMode() {
    BindingMode selectedBindingMode = bindingModeChooser.get();
    return selectedBindingMode != null ? selectedBindingMode : BindingMode.COMPETITION;
  }

  private void applyCompetitionDefaults() {

    // turret1.setDefaultCommand(
    //     new TurretTargeting(turret1, drive, robotToTurret1, Turret.getTargetingMode()));
    // turret2.setDefaultCommand(
    //     new TurretTargeting(turret2, drive, robotToTurret2, Turret.getTargetingMode()));
    // Re-enable hub tracking by commenting out the two lock lines above and uncommenting the two
    // lines below.
    turret1.setDefaultCommand(new TurretTargeting(turret1, drive, robotToTurret1));
    turret2.setDefaultCommand(new TurretTargeting(turret2, drive, robotToTurret2));
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
  }
}
