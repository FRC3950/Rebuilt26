package frc.robot.controls;

import static frc.robot.Constants.SubsystemConstants.Intake.downPos;
import static frc.robot.Constants.SubsystemConstants.Intake.upPos;
import static frc.robot.Constants.SubsystemConstants.Turret.maxAzimuthAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.maxHoodAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.minAzimuthAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.minHoodAngle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;

public final class TuneModeBindings {
  private static final String LEFT_PREFIX = "Tune/Left";
  private static final String RIGHT_PREFIX = "Tune/Right";
  private static final String ANGLE_SUFFIX = " Turret Angle Deg";
  private static final String HOOD_SUFFIX = " Hood Deg";
  private static final String FLYWHEEL_SUFFIX = " Flywheel RPS";
  private static final String VALID_SUFFIX = " Valid";
  private static final String STATUS_SUFFIX = " Status";

  private TuneModeBindings() {}

  public static void configure(
      EventLoop buttonLoop,
      CommandXboxController driver,
      Drive drive,
      Intake intake,
      Indexer indexer,
      Turret leftTurret,
      Turret rightTurret) {
    publishDefaultTuneValues(LEFT_PREFIX, leftTurret);
    publishDefaultTuneValues(RIGHT_PREFIX, rightTurret);

    driver
        .leftTrigger(0.5, buttonLoop)
        .whileTrue(
            new IntakeCommand(intake, indexer, driver.rightTrigger(0.5, buttonLoop)::getAsBoolean));

    driver
        .y(buttonLoop)
        .and(driver.start(buttonLoop).negate())
        .onTrue(Commands.runOnce(() -> toggleIntakePivot(intake), intake));

    driver.rightTrigger(0.5, buttonLoop).whileTrue(indexer.feedCommand());

    driver
        .leftBumper(buttonLoop)
        .whileTrue(
            Commands.run(
                () -> {
                  applyTurretTune(LEFT_PREFIX, leftTurret);
                  applyTurretTune(RIGHT_PREFIX, rightTurret);
                },
                leftTurret,
                rightTurret));

    driver
        .start(buttonLoop)
        .and(driver.a(buttonLoop))
        .whileTrue(drive.sysIdTranslationQuasistatic(SysIdRoutine.Direction.kForward));
    driver
        .start(buttonLoop)
        .and(driver.b(buttonLoop))
        .whileTrue(drive.sysIdTranslationDynamic(SysIdRoutine.Direction.kForward));
    driver
        .start(buttonLoop)
        .and(driver.x(buttonLoop))
        .whileTrue(drive.sysIdRotationQuasistatic(SysIdRoutine.Direction.kForward));
    driver
        .start(buttonLoop)
        .and(driver.y(buttonLoop))
        .whileTrue(drive.sysIdRotationDynamic(SysIdRoutine.Direction.kForward));
  }

  public static edu.wpi.first.wpilibj2.command.Command createDriveDefaultCommand(
      Drive drive, CommandXboxController driver) {
    return DriveCommands.joystickDrive(
        drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX());
  }

  static TurretTuneSetpoint validateTuneSetpoint(
      double turretAngleDeg, double hoodAngleDeg, double flywheelRps) {
    if (!Double.isFinite(turretAngleDeg)) {
      return new TurretTuneSetpoint(turretAngleDeg, hoodAngleDeg, flywheelRps, false, "Angle NaN");
    }
    if (turretAngleDeg < minAzimuthAngle || turretAngleDeg > maxAzimuthAngle) {
      return new TurretTuneSetpoint(
          turretAngleDeg, hoodAngleDeg, flywheelRps, false, "Angle out of range");
    }
    if (!Double.isFinite(hoodAngleDeg)) {
      return new TurretTuneSetpoint(turretAngleDeg, hoodAngleDeg, flywheelRps, false, "Hood NaN");
    }
    if (hoodAngleDeg < minHoodAngle || hoodAngleDeg > maxHoodAngle) {
      return new TurretTuneSetpoint(
          turretAngleDeg, hoodAngleDeg, flywheelRps, false, "Hood out of range");
    }
    if (!Double.isFinite(flywheelRps)) {
      return new TurretTuneSetpoint(
          turretAngleDeg, hoodAngleDeg, flywheelRps, false, "Flywheel NaN");
    }
    if (flywheelRps < 0.0) {
      return new TurretTuneSetpoint(
          turretAngleDeg, hoodAngleDeg, flywheelRps, false, "Flywheel must be non-negative");
    }
    return new TurretTuneSetpoint(turretAngleDeg, hoodAngleDeg, flywheelRps, true, "OK");
  }

  static double getIntakeToggleTarget(double currentPivotPosition) {
    double midpoint = (upPos + downPos) * 0.5;
    return currentPivotPosition <= midpoint ? upPos : downPos;
  }

  private static void toggleIntakePivot(Intake intake) {
    intake.setPivotPosition(getIntakeToggleTarget(intake.getPivotPosition()));
  }

  private static void publishDefaultTuneValues(String prefix, Turret turret) {
    SmartDashboard.setDefaultNumber(prefix + ANGLE_SUFFIX, 0.0);
    SmartDashboard.setDefaultNumber(prefix + HOOD_SUFFIX, turret.getHoodSetpointDeg());
    SmartDashboard.setDefaultNumber(prefix + FLYWHEEL_SUFFIX, 0.0);
    SmartDashboard.putBoolean(prefix + VALID_SUFFIX, true);
    SmartDashboard.putString(prefix + STATUS_SUFFIX, "Idle");
  }

  private static void applyTurretTune(String prefix, Turret turret) {
    TurretTuneSetpoint setpoint =
        validateTuneSetpoint(
            SmartDashboard.getNumber(prefix + ANGLE_SUFFIX, 0.0),
            SmartDashboard.getNumber(prefix + HOOD_SUFFIX, turret.getHoodSetpointDeg()),
            SmartDashboard.getNumber(prefix + FLYWHEEL_SUFFIX, 0.0));

    SmartDashboard.putBoolean(prefix + VALID_SUFFIX, setpoint.valid());
    SmartDashboard.putString(prefix + STATUS_SUFFIX, setpoint.status());

    if (!setpoint.valid()) {
      turret.stopFlywheel();
      return;
    }

    turret.runSetpoints(
        Rotation2d.fromDegrees(setpoint.turretAngleDeg()),
        setpoint.hoodAngleDeg(),
        setpoint.flywheelRps());
  }

  record TurretTuneSetpoint(
      double turretAngleDeg,
      double hoodAngleDeg,
      double flywheelRps,
      boolean valid,
      String status) {}
}
