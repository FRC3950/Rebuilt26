package frc.robot.controls;

import static frc.robot.Constants.FieldConstants.hubTranslation;
import static frc.robot.Constants.SubsystemConstants.Intake.downPos;
import static frc.robot.Constants.SubsystemConstants.Intake.upPos;
import static frc.robot.Constants.SubsystemConstants.Turret.maxHoodAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.minHoodAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret1;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private static final String SHARED_HOOD_KEY = "Tune/Hood Deg";
  private static final String SHARED_FLYWHEEL_KEY = "Tune/Flywheel RPS";
  private static final String SHARED_VALID_KEY = "Tune/Setpoints Valid";
  private static final String SHARED_STATUS_KEY = "Tune/Setpoints Status";
  private static final String LEFT_TURRET_DISTANCE_KEY = "Tune/Left Turret To Hub Distance M";

  private TuneModeBindings() {}

  public static void configure(
      EventLoop buttonLoop,
      CommandXboxController driver,
      Drive drive,
      Intake intake,
      Indexer indexer,
      Turret leftTurret,
      Turret rightTurret) {
    publishDefaultTuneValues(leftTurret);

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
                  TurretTuneSetpoint setpoint =
                      validateTuneSetpoint(
                          SmartDashboard.getNumber(
                              SHARED_HOOD_KEY, leftTurret.getHoodSetpointDeg()),
                          SmartDashboard.getNumber(SHARED_FLYWHEEL_KEY, 0.0));

                  SmartDashboard.putBoolean(SHARED_VALID_KEY, setpoint.valid());
                  SmartDashboard.putString(SHARED_STATUS_KEY, setpoint.status());

                  if (!setpoint.valid()) {
                    leftTurret.stopFlywheel();
                    rightTurret.stopFlywheel();
                    return;
                  }

                  applyTurretTune(leftTurret, drive.getPose(), robotToTurret1, setpoint);
                  applyTurretTune(rightTurret, drive.getPose(), robotToTurret2, setpoint);
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

  public static void publishTuneTelemetry(Pose2d robotPose) {
    SmartDashboard.putNumber(LEFT_TURRET_DISTANCE_KEY, getDistanceToHub(robotPose, robotToTurret1));
  }

  static TurretTuneSetpoint validateTuneSetpoint(double hoodAngleDeg, double flywheelRps) {
    if (!Double.isFinite(hoodAngleDeg)) {
      return new TurretTuneSetpoint(hoodAngleDeg, flywheelRps, false, "Hood NaN");
    }
    if (hoodAngleDeg < minHoodAngle || hoodAngleDeg > maxHoodAngle) {
      return new TurretTuneSetpoint(hoodAngleDeg, flywheelRps, false, "Hood out of range");
    }
    if (!Double.isFinite(flywheelRps)) {
      return new TurretTuneSetpoint(hoodAngleDeg, flywheelRps, false, "Flywheel NaN");
    }
    if (flywheelRps < 0.0) {
      return new TurretTuneSetpoint(
          hoodAngleDeg, flywheelRps, false, "Flywheel must be non-negative");
    }
    return new TurretTuneSetpoint(hoodAngleDeg, flywheelRps, true, "OK");
  }

  static double getIntakeToggleTarget(double currentPivotPosition) {
    double midpoint = (upPos + downPos) * 0.5;
    return currentPivotPosition <= midpoint ? upPos : downPos;
  }

  private static void toggleIntakePivot(Intake intake) {
    intake.setPivotPosition(getIntakeToggleTarget(intake.getPivotPosition()));
  }

  private static void publishDefaultTuneValues(Turret turret) {
    SmartDashboard.setDefaultNumber(SHARED_HOOD_KEY, turret.getHoodSetpointDeg());
    SmartDashboard.setDefaultNumber(SHARED_FLYWHEEL_KEY, 0.0);
    SmartDashboard.putBoolean(SHARED_VALID_KEY, true);
    SmartDashboard.putString(SHARED_STATUS_KEY, "Idle");
    SmartDashboard.putNumber(LEFT_TURRET_DISTANCE_KEY, 0.0);
  }

  private static void applyTurretTune(
      Turret turret, Pose2d robotPose, Translation2d robotToTurret, TurretTuneSetpoint setpoint) {
    turret.runSetpoints(
        getHubHeadingRobot(robotPose, robotToTurret),
        setpoint.hoodAngleDeg(),
        setpoint.flywheelRps());
  }

  static double getDistanceToHub(Pose2d robotPose, Translation2d robotToTurret) {
    return getTurretFieldPosition(robotPose, robotToTurret).getDistance(hubTranslation);
  }

  private static Rotation2d getHubHeadingRobot(Pose2d robotPose, Translation2d robotToTurret) {
    Rotation2d fieldHeading =
        hubTranslation.minus(getTurretFieldPosition(robotPose, robotToTurret)).getAngle();
    return fieldHeading.minus(robotPose.getRotation());
  }

  private static Translation2d getTurretFieldPosition(
      Pose2d robotPose, Translation2d robotToTurret) {
    return robotPose.getTranslation().plus(robotToTurret.rotateBy(robotPose.getRotation()));
  }

  record TurretTuneSetpoint(
      double hoodAngleDeg, double flywheelRps, boolean valid, String status) {}
}
