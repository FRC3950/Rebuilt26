package frc.robot;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.SubsystemConstants.*;
import static frc.robot.Constants.SubsystemConstants.Turret.*;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommand;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.SmartShotRelease;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
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

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
            hoodID,
            hoodConfig,
            flywheelID,
            flywheelConfig,
            flywheelFollowerID,
            CANivore);
    turret2 =
        new Turret(
            azimuthID2,
            azimuthConfig,
            hoodID2,
            hoodConfig,
            flywheelID2,
            flywheelConfig,
            flywheelFollowerID2,
            CANivore);

    intake = new Intake();
    indexer = new Indexer();
    climber = new Climber();

    turret1.setDefaultCommand(new TurretTargeting(turret1, drive, robotToTurret1));
    turret2.setDefaultCommand(new TurretTargeting(turret2, drive, robotToTurret2));
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

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Primary Driver Layout: https://tinyurl.com/5n7kv2up

    // Snake Mode
    SmartDashboard.setDefaultBoolean("Snake Mode", true);

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> SmartDashboard.getBoolean("Snake Mode", true),
            intake::isIntaking));

    // Auto Ferry Mode
    new Trigger(
            () ->
                drive.getPose().getX() > neutralZoneMinX
                    && drive.getPose().getX() < neutralZoneMaxX)
        .whileTrue(new FerryMode(turret1, drive, robotToTurret1));

    new Trigger(
            () ->
                drive.getPose().getX() > neutralZoneMinX
                    && drive.getPose().getX() < neutralZoneMaxX)
        .whileTrue(new FerryMode(turret2, drive, robotToTurret2));

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
        .whileTrue(indexer.feedCommand());

    // Climber
    driver.povDown().debounce(0.75).onTrue(climber.togglePositionCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
