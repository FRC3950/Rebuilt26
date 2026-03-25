package frc.robot;

import static frc.robot.Constants.FieldConstants.hubTranslation;
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
import static frc.robot.Constants.SubsystemConstants.Turret.maxFlywheelRps;
import static frc.robot.Constants.SubsystemConstants.Turret.maxHoodAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.minFlywheelRps;
import static frc.robot.Constants.SubsystemConstants.Turret.minHoodAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.rightAzimuthConfig;
import static frc.robot.Constants.SubsystemConstants.Turret.rightMaxAzimuthControlAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.rightMinAzimuthControlAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret1;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.subsystems.turret.TurretTargeting.OutputMode;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  static final String LEFT_TURRET_HOOD_ANGLE_KEY = "Left Turret Hood Angle Deg";
  static final String LEFT_TURRET_FLYWHEEL_RPS_KEY = "Left Turret Flywheel RPS";
  static final String RUN_LEFT_TURRET_KEY = "Run Left Turret";
  static final String LEFT_TURRET_DISTANCE_TO_HUB_KEY = "Left Turret Distance To Hub M";
  static final String TARGET_LEFT_TURRET_DISTANCE_TO_HUB_KEY =
      "Target Left Turret Distance To Hub M";
  static final String SET_LEFT_TURRET_DISTANCE_KEY = "Set Left Turret Distance";

  private final Drive drive;
  private final Turret turret1;
  private final Turret turret2;
  private final Vision vision;
  private final Intake intake;
  private final Indexer indexer;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final LoggedDashboardChooser<Command> autoChooser;
  private boolean previousSetLeftTurretDistance = false;

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
            leftAzimuthConfig,
            leftMinAzimuthControlAngle,
            leftMaxAzimuthControlAngle,
            HOOD_SERVO_CHANNEL_1,
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
            HOOD_SERVO_CHANNEL_2,
            flywheelID2,
            flywheelConfig,
            flywheelFollowerID2,
            CANivore);

    intake = new Intake();
    indexer = new Indexer();

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
    SmartDashboard.putNumber(LEFT_TURRET_HOOD_ANGLE_KEY, minHoodAngle);
    SmartDashboard.putNumber(LEFT_TURRET_FLYWHEEL_RPS_KEY, minFlywheelRps);
    SmartDashboard.putBoolean(RUN_LEFT_TURRET_KEY, false);
    SmartDashboard.putNumber(LEFT_TURRET_DISTANCE_TO_HUB_KEY, getLeftTurretDistanceToHubMeters());
    SmartDashboard.putNumber(
        TARGET_LEFT_TURRET_DISTANCE_TO_HUB_KEY, getLeftTurretDistanceToHubMeters());
    SmartDashboard.putBoolean(SET_LEFT_TURRET_DISTANCE_KEY, false);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices: ", AutoBuilder.buildAutoChooser());

    configureBindings();
    configureDefaultCommands();
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void periodic() {
    SmartDashboard.putNumber(LEFT_TURRET_DISTANCE_TO_HUB_KEY, getLeftTurretDistanceToHubMeters());

    boolean setLeftTurretDistance = getSetLeftTurretDistance();
    if (shouldStartDistanceCommand(previousSetLeftTurretDistance, setLeftTurretDistance)) {
      scheduleLeftTurretDistanceCommand();
    }
    previousSetLeftTurretDistance = setLeftTurretDistance;
  }

  private void configureBindings() {
    operator
        .leftTrigger(0.5)
        .whileTrue(
            new IntakeCommand(
                intake,
                () -> {
                  var speeds = drive.getRobotRelativeSpeeds();
                  return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
                }));

    operator.rightBumper().onTrue(intake.retractCommand());

    operator
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

    driver.a().whileTrue(DriveCommands.alignRobotToHubLine(drive, robotToTurret1));
    driver
        .povUp()
        .whileTrue(
            DriveCommands.robotRelativeNudge(
                drive, DriveCommands.getDpadNudgeSpeedMetersPerSecond(), 0.0));
    driver
        .povDown()
        .whileTrue(
            DriveCommands.robotRelativeNudge(
                drive, -DriveCommands.getDpadNudgeSpeedMetersPerSecond(), 0.0));
    driver
        .povLeft()
        .whileTrue(
            DriveCommands.robotRelativeNudge(
                drive, 0.0, DriveCommands.getDpadNudgeSpeedMetersPerSecond()));
    driver
        .povRight()
        .whileTrue(
            DriveCommands.robotRelativeNudge(
                drive, 0.0, -DriveCommands.getDpadNudgeSpeedMetersPerSecond()));

    operator
        .b()
        .and(operator.start().negate())
        .whileTrue(Commands.startEnd(() -> intake.setIntakeSpeed(-45), intake::stopIntake, intake));

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
  }

  private void configureDefaultCommands() {
    turret1.setDefaultCommand(
        new TurretTargeting(
            turret1,
            drive,
            robotToTurret1,
            OutputMode.MANUAL_SETPOINTS_WHEN_ENABLED,
            this::getLeftTurretHoodAngleDeg,
            this::getLeftTurretFlywheelRps,
            this::getRunLeftTurret));
    turret2.setDefaultCommand(new TurretTargeting(turret2, drive, robotToTurret2));
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
  }

  private double getLeftTurretHoodAngleDeg() {
    return MathUtil.clamp(
        SmartDashboard.getNumber(LEFT_TURRET_HOOD_ANGLE_KEY, minHoodAngle),
        minHoodAngle,
        maxHoodAngle);
  }

  private double getLeftTurretFlywheelRps() {
    return MathUtil.clamp(
        SmartDashboard.getNumber(LEFT_TURRET_FLYWHEEL_RPS_KEY, minFlywheelRps),
        minFlywheelRps,
        maxFlywheelRps);
  }

  private boolean getRunLeftTurret() {
    return SmartDashboard.getBoolean(RUN_LEFT_TURRET_KEY, false);
  }

  private double getLeftTurretDistanceToHubMeters() {
    return TurretTargeting.getDistanceToTargetMeters(
        drive.getPose(), robotToTurret1, hubTranslation);
  }

  private double getTargetLeftTurretDistanceToHubMeters() {
    return Math.max(
        0.0,
        SmartDashboard.getNumber(
            TARGET_LEFT_TURRET_DISTANCE_TO_HUB_KEY, getLeftTurretDistanceToHubMeters()));
  }

  private boolean getSetLeftTurretDistance() {
    return SmartDashboard.getBoolean(SET_LEFT_TURRET_DISTANCE_KEY, false);
  }

  private void scheduleLeftTurretDistanceCommand() {
    Command distanceCommand =
        DriveCommands.driveLeftTurretToHubDistance(
                drive, robotToTurret1, getTargetLeftTurretDistanceToHubMeters())
            .finallyDo(
                () -> {
                  SmartDashboard.putBoolean(SET_LEFT_TURRET_DISTANCE_KEY, false);
                  previousSetLeftTurretDistance = false;
                });
    CommandScheduler.getInstance().schedule(distanceCommand);
  }

  static boolean shouldStartDistanceCommand(boolean previousToggle, boolean currentToggle) {
    return !previousToggle && currentToggle;
  }
}
