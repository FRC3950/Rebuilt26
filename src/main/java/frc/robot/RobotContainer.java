package frc.robot;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.SubsystemConstants.*;
import static frc.robot.Constants.SubsystemConstants.Turret.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.turret.FerryMode;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretTargeting;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Turret turret1;
  private final Turret turret2;

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
    // Primary Driver Layout: https://tinyurl.com/2uptvdyh

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // Auto Ferry Mode
    new Trigger(
            () ->
                drive.getPose().getY() > Constants.FieldConstants.neutralZoneMinY
                    && drive.getPose().getY() < Constants.FieldConstants.neutralZoneMaxY)
        .whileTrue(new FerryMode(turret1, drive, robotToTurret1));

    new Trigger(
            () ->
                drive.getPose().getY() > Constants.FieldConstants.neutralZoneMinY
                    && drive.getPose().getY() < Constants.FieldConstants.neutralZoneMaxY)
        .whileTrue(new FerryMode(turret2, drive, robotToTurret2));

    // Manual Ferry Mode - Left
    driver
        .povLeft()
        .and(() -> drive.getPose().getY() > Constants.FieldConstants.neutralZoneMinY)
        .whileTrue(
            new FerryMode(turret1, drive, robotToTurret1, leftFerryTarget)
                .alongWith(new FerryMode(turret2, drive, robotToTurret2, leftFerryTarget)));

    // Manual Ferry Mode - Right
    driver
        .povRight()
        .and(() -> drive.getPose().getY() > neutralZoneMinY)
        .whileTrue(
            new FerryMode(turret1, drive, robotToTurret1, rightFerryTarget)
                .alongWith(new FerryMode(turret2, drive, robotToTurret2, rightFerryTarget)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
