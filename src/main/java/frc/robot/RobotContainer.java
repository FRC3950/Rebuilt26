package frc.robot;

import static frc.robot.Constants.SubsystemConstants.CANivore;
import static frc.robot.Constants.SubsystemConstants.Turret.HOOD_SERVO_CHANNEL_1;
import static frc.robot.Constants.SubsystemConstants.Turret.HOOD_SERVO_CHANNEL_2;
import static frc.robot.Constants.SubsystemConstants.Turret.HOOD_SERVO_HUB_CAN_ID;
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

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.servohub.ServoChannel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.controls.CompBindings;
import frc.robot.controls.sim.RobotSimulationControls;
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
import frc.robot.subsystems.turret.TurretTuningDashboard;
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
import frc.robot.util.MirroringAutoChooser;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  public enum CodeMode {
    COMPETITION,
    DEMO
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
  private final RobotSimulationControls robotSimulationControls;
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final EventLoop competitionButtonLoop = new EventLoop();
  private static final int MIN_REV_CAN_ID = 0;
  private static final int MAX_REV_CAN_ID = 62;

  private final MirroringAutoChooser autoChooser;
  private final LoggedDashboardChooser<CodeMode> codeModeChooser;

  private DemoContainer demoContainer = null;
  private CodeMode appliedCodeMode = CodeMode.COMPETITION;
  private CodeMode lastPublishedSelectedCodeMode = null;

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
    vision.setPoseFusionAllowedSupplier(
        () ->
            appliedCodeMode != CodeMode.DEMO
                || demoContainer == null
                || demoContainer.isVisionPoseFusionAllowed());
    robotSimulationControls =
        new RobotSimulationControls(() -> applyCodeMode(CodeMode.DEMO), () -> demoContainer);
    indexer.setForwardFeedAllowedSupplier(
        () ->
            Constants.currentMode != Constants.Mode.REAL
                || (turret1.isFlywheelReadyForFeed() && turret2.isFlywheelReadyForFeed()));

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
    NamedCommands.registerCommand("X Wheel Lock", Commands.runOnce(drive::stopWithX, drive));
    NamedCommands.registerCommand(
        "Intake While Held", Commands.startEnd(intake::startIntake, intake::stopIntake, intake));
    NamedCommands.registerCommand("Start Hotdog", Commands.runOnce(indexer::startHotdog, indexer));
    NamedCommands.registerCommand("Stop Hotdog", Commands.runOnce(indexer::stopHotdog, indexer));

    NamedCommands.registerCommand(
        "Start Shoot",
        Commands.runOnce(
            () -> {
              indexer.requestForwardFeed();
              intake.startIntake();
            },
            indexer,
            intake));
    NamedCommands.registerCommand(
        "End Shoot",
        Commands.runOnce(
            () -> {
              indexer.stopForwardFeed();
              intake.stopIntake();
            },
            indexer,
            intake));

    SmartDashboard.putData("Turret Subsystem", turret1);
    TurretTuningDashboard.register(turret1, turret2);
    autoChooser = new MirroringAutoChooser("Auto Choices: ");

    codeModeChooser = new LoggedDashboardChooser<>("Code Mode");
    codeModeChooser.addDefaultOption("Competition", CodeMode.COMPETITION);
    codeModeChooser.addOption("Demo", CodeMode.DEMO);

    configureCompetitionBindings();
    applyCompetitionDefaults();
    applyCodeMode(CodeMode.COMPETITION);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getAutonomousCommand();
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
    CodeMode selectedCodeMode = getSelectedCodeMode();
    publishSelectedCodeMode(selectedCodeMode);

    if (shouldApplyCodeMode(selectedCodeMode, appliedCodeMode, DriverStation.isDisabled())) {
      applyCodeMode(selectedCodeMode);
      return;
    }

    if (appliedCodeMode == CodeMode.DEMO && demoContainer != null) {
      demoContainer.checkMode(DriverStation.isDisabled());
    }
  }

  static boolean shouldApplyCodeMode(
      CodeMode selectedCodeMode, CodeMode currentCodeMode, boolean isDisabled) {
    return isDisabled && selectedCodeMode != currentCodeMode;
  }

  private void configureCompetitionBindings() {
    CompBindings.configure(
        competitionButtonLoop, driver, operator, drive, intake, indexer, turret1, turret2);
  }

  private Turret createRealLeftTurret() {
    return new Turret(
        "LeftTurret",
        new Azimuth(
            "Turret/Left/Azimuth",
            new AzimuthIOTalonFX(azimuthID, leftAzimuthConfig, CANivore, true)),
        createRealHood("Turret/Left/Hood", HOOD_SERVO_CHANNEL_2),
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
        createRealHood("Turret/Right/Hood", HOOD_SERVO_CHANNEL_1),
        new Flywheels(
            "Turret/Right/Flywheels",
            new FlywheelsIOTalonFX(flywheelID2, flywheelConfig, flywheelFollowerID2, CANivore)),
        rightMinAzimuthControlAngle,
        rightMaxAzimuthControlAngle);
  }

  private Hood createRealHood(String logKey, ServoChannel.ChannelId channelId) {
    if (!isValidRevServoHubCanId(HOOD_SERVO_HUB_CAN_ID)) {
      DriverStation.reportError(
          "REV Servo Hub CAN ID "
              + HOOD_SERVO_HUB_CAN_ID
              + " is invalid for REVLib; valid IDs are "
              + MIN_REV_CAN_ID
              + "-"
              + MAX_REV_CAN_ID
              + ". "
              + logKey
              + " will be disabled until the Servo Hub CAN ID is changed.",
          false);
      return new Hood(logKey, new HoodIO() {});
    }

    try {
      return new Hood(logKey, new HoodIOServoHub(channelId));
    } catch (RuntimeException ex) {
      DriverStation.reportError(
          "Failed to initialize " + logKey + "; hood servo output disabled: " + ex.getMessage(),
          ex.getStackTrace());
      return new Hood(logKey, new HoodIO() {});
    }
  }

  static boolean isValidRevServoHubCanId(int canId) {
    return canId >= MIN_REV_CAN_ID && canId <= MAX_REV_CAN_ID;
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

  private void applyCodeMode(CodeMode codeMode) {
    switch (codeMode) {
      case COMPETITION:
        drive.setMaxLinearSpeedSupplier(drive::getPhysicalMaxLinearSpeedMetersPerSec);
        drive.setReducedSpeedSupplier(() -> Constants.SubsystemConstants.Drive.reducedSpeed);
        applyCompetitionDefaults();
        CommandScheduler.getInstance().setActiveButtonLoop(competitionButtonLoop);
        break;
      case DEMO:
        if (demoContainer == null) {
          demoContainer =
              new DemoContainer(driver, operator, drive, vision, intake, indexer, turret1, turret2);
        }
        drive.setMaxLinearSpeedSupplier(demoContainer::getMaxDriveSpeedMetersPerSec);
        drive.setReducedSpeedSupplier(demoContainer::getReducedSpeedMetersPerSec);
        demoContainer.applyDefaults();
        demoContainer.applyCurrentBindingMode();
        break;
    }

    appliedCodeMode = codeMode;
    SmartDashboard.putString("Code Mode/Applied", appliedCodeMode.name());
    Logger.recordOutput("Controls/CodeModeApplied", appliedCodeMode.name());
  }

  private void publishSelectedCodeMode(CodeMode selectedCodeMode) {
    if (selectedCodeMode == lastPublishedSelectedCodeMode) {
      return;
    }

    lastPublishedSelectedCodeMode = selectedCodeMode;
    SmartDashboard.putString("Code Mode/Selected", selectedCodeMode.name());
    Logger.recordOutput("Controls/CodeModeSelected", selectedCodeMode.name());
  }

  private CodeMode getSelectedCodeMode() {
    CodeMode selectedCodeMode = codeModeChooser.get();
    return selectedCodeMode != null ? selectedCodeMode : CodeMode.COMPETITION;
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
