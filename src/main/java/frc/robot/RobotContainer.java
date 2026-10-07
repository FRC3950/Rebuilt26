package frc.robot;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommand;
import frc.robot.controls.DefaultModeBindings;
import frc.robot.controls.TuneModeBindings;
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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretTargeting;
import frc.robot.subsystems.turret.TurretVisualization;
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
    DEFAULT,
    TUNE
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

  private final EventLoop defaultButtonLoop = new EventLoop();
  private final EventLoop tuneButtonLoop = new EventLoop();

  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<BindingMode> bindingModeChooser;

  private BindingMode appliedBindingMode = BindingMode.DEFAULT;

  public RobotContainer() {
    intake = new Intake();
    indexer = new Indexer();
    switch (Constants.currentMode) {
      case REAL:
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
        break;

      case SIM:
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
        break;

      default:
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
        break;
    }

    turret1 =
        new Turret(
            azimuthID,
            leftAzimuthConfig,
            leftMinAzimuthControlAngle,
            leftMaxAzimuthControlAngle,
            Constants.SubsystemConstants.Turret.leftAzimuthEncoder,
            Constants.SubsystemConstants.Turret.leftAzimuthStartupCenterDeg,
            HOOD_SERVO_CHANNEL_2,
            flywheelID,
            flywheelConfig,
            flywheelFollowerID,
            CANivore);
    turret2 =
        new Turret(
            azimuthID2,
            rightAzimuthConfig,
            rightMinAzimuthControlAngle,
            rightMaxAzimuthControlAngle,
            Constants.SubsystemConstants.Turret.rightAzimuthEncoder,
            Constants.SubsystemConstants.Turret.rightAzimuthStartupCenterDeg,
            HOOD_SERVO_CHANNEL_1,
            flywheelID2,
            flywheelConfig,
            flywheelFollowerID2,
            CANivore);
    turret1.setStartupInterlock(() -> turret1.isStartupReady() && turret2.isStartupReady());
    turret2.setStartupInterlock(() -> turret1.isStartupReady() && turret2.isStartupReady());
    SmartDashboard.putData(
        "Turrets/Confirm both startup windows",
        Commands.runOnce(
                () -> {
                  if (DriverStation.isDisabled()) {
                    turret1.initializeFromStartupWindow();
                    turret2.initializeFromStartupWindow();
                  }
                },
                turret1,
                turret2)
            .ignoringDisable(true));
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
    bindingModeChooser.addDefaultOption("Default", BindingMode.DEFAULT);
    bindingModeChooser.addOption("TUNE", BindingMode.TUNE);

    DefaultModeBindings.configure(
        defaultButtonLoop, driver, drive, intake, indexer, turret1, turret2);
    configureEmergencyBindings();
    configureTuneBindings();
    configureDefaults();
    applyBindingMode(BindingMode.DEFAULT);
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
    SmartDashboard.putString("Code Mode/Selected", selectedBindingMode.name());
    Logger.recordOutput("Controls/BindingModeSelected", selectedBindingMode.name());

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

  private void configureEmergencyBindings() {
    operator
        .leftTrigger(0.5, defaultButtonLoop)
        .whileTrue(
            new IntakeCommand(
                intake,
                () -> {
                  var speeds = drive.getRobotRelativeSpeeds();
                  return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
                }));
    operator.rightBumper(defaultButtonLoop).onTrue(intake.retractCommand());

    operator
        .rightTrigger(0.5, defaultButtonLoop)
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
        .b(defaultButtonLoop)
        .and(operator.start(defaultButtonLoop).negate())
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
    operator
        .start(defaultButtonLoop)
        .and(operator.b(defaultButtonLoop))
        .whileTrue(Commands.startEnd(intake::reverseIntake, intake::stopIntake, intake));

    operator.a(defaultButtonLoop).onTrue(Commands.runOnce(Turret::toggleTurretMode)).debounce(0.25);
  }

  private void configureTuneBindings() {
    TuneModeBindings.configure(tuneButtonLoop, driver, drive, intake, indexer, turret1, turret2);
  }

  private void applyBindingMode(BindingMode bindingMode) {
    CommandScheduler.getInstance()
        .setActiveButtonLoop(
            switch (bindingMode) {
              case DEFAULT -> defaultButtonLoop;
              case TUNE -> tuneButtonLoop;
            });

    TuneModeBindings.setDashboardActive(
        bindingMode == BindingMode.TUNE, turret1.getCommandedHoodAngleDeg());
    appliedBindingMode = bindingMode;
    SmartDashboard.putString("Code Mode/Applied", appliedBindingMode.name());
    Logger.recordOutput("Controls/BindingModeApplied", appliedBindingMode.name());
  }

  private BindingMode getSelectedBindingMode() {
    BindingMode selectedBindingMode = bindingModeChooser.get();
    return selectedBindingMode != null ? selectedBindingMode : BindingMode.DEFAULT;
  }

  private void configureDefaults() {

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
