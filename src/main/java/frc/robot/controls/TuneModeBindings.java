package frc.robot.controls;

import static frc.robot.Constants.FieldConstants.getHubTranslation;
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
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.GetAdjustedShot;
import frc.robot.subsystems.turret.Turret;

/**
 * Shot table tuning. Both turrets share one hood/flywheel setpoint because they read the same
 * shot_table.json, so record a row using the distance of the turret whose shot you were watching.
 */
public final class TuneModeBindings {
  private static final String SHARED_HOOD_KEY = "Tune/Hood Deg";
  private static final String SHARED_FLYWHEEL_KEY = "Tune/Flywheel RPS";
  private static final String SHARED_VALID_KEY = "Tune/Setpoints Valid";
  private static final String SHARED_STATUS_KEY = "Tune/Setpoints Status";
  private static final String LEFT_TURRET_DISTANCE_KEY = "Tune/Left Turret To Hub Distance M";
  private static final String RIGHT_TURRET_DISTANCE_KEY = "Tune/Right Turret To Hub Distance M";
  private static final String SAVE_STATUS_KEY = "Tune/Save Status";

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
    buttonLoop.bind(() -> publishTuneTelemetry(drive.getPose()));

    driver
        .leftTrigger(0.5, buttonLoop)
        .whileTrue(
            new IntakeCommand(
                intake,
                () -> {
                  var speeds = drive.getRobotRelativeSpeeds();
                  return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
                }));

    driver.rightBumper(buttonLoop).onTrue(intake.retractCommand());

    driver.rightTrigger(0.5, buttonLoop).whileTrue(indexer.feedCommand());

    driver
        .y(buttonLoop)
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));

    driver
        .leftBumper(buttonLoop)
        .whileTrue(
            Commands.run(
                () -> {
                  TurretTuneSetpoint setpoint =
                      validateTuneSetpoint(
                          SmartDashboard.getNumber(
                              SHARED_HOOD_KEY, leftTurret.getCommandedHoodAngleDeg()),
                          SmartDashboard.getNumber(SHARED_FLYWHEEL_KEY, 0.0));

                  SmartDashboard.putBoolean(SHARED_VALID_KEY, setpoint.valid());
                  SmartDashboard.putString(SHARED_STATUS_KEY, setpoint.status());

                  if (!setpoint.valid()) {
                    leftTurret.stop();
                    rightTurret.stop();
                    return;
                  }

                  applyTurretTune(leftTurret, drive.getPose(), robotToTurret1, setpoint);
                  applyTurretTune(rightTurret, drive.getPose(), robotToTurret2, setpoint);
                },
                leftTurret,
                rightTurret));

    driver
        .a(buttonLoop)
        .onTrue(Commands.runOnce(() -> saveTunePoint(drive.getPose())).ignoringDisable(true));
  }

  private static void saveTunePoint(Pose2d robotPose) {
    TurretTuneSetpoint setpoint =
        validateTuneSetpoint(
            SmartDashboard.getNumber(SHARED_HOOD_KEY, Double.NaN),
            SmartDashboard.getNumber(SHARED_FLYWHEEL_KEY, 0.0));
    if (!setpoint.valid()) {
      SmartDashboard.putString(SAVE_STATUS_KEY, "Not saved: " + setpoint.status());
      return;
    }
    if (setpoint.flywheelRps() <= 0.0) {
      SmartDashboard.putString(SAVE_STATUS_KEY, "Not saved: flywheel is 0");
      return;
    }

    // Both turrets share one table row, so save the distance between them. Line up head-on to the
    // hub so the two turret distances match.
    double distanceMeters =
        (getDistanceToHub(robotPose, robotToTurret1) + getDistanceToHub(robotPose, robotToTurret2))
            / 2.0;
    boolean written =
        GetAdjustedShot.saveTunedRow(
            distanceMeters, setpoint.hoodAngleDeg(), setpoint.flywheelRps());

    SmartDashboard.putString(
        SAVE_STATUS_KEY,
        String.format(
            "%s d=%.2f m, hood=%.1f, rps=%.1f",
            written ? "Saved" : "In use but FILE WRITE FAILED:",
            distanceMeters,
            setpoint.hoodAngleDeg(),
            setpoint.flywheelRps()));
  }

  private static void publishTuneTelemetry(Pose2d robotPose) {
    SmartDashboard.putNumber(LEFT_TURRET_DISTANCE_KEY, getDistanceToHub(robotPose, robotToTurret1));
    SmartDashboard.putNumber(
        RIGHT_TURRET_DISTANCE_KEY, getDistanceToHub(robotPose, robotToTurret2));
  }

  static TurretTuneSetpoint validateTuneSetpoint(double hoodAngleDeg, double flywheelRps) {
    if (!Double.isFinite(hoodAngleDeg)) {
      return new TurretTuneSetpoint(hoodAngleDeg, flywheelRps, false, "Hood NaN");
    }
    // Dashboard widgets can land a hair past a limit, e.g. 12.9999 still displays as 13.
    double roundedHoodDeg = Math.round(hoodAngleDeg * 100.0) / 100.0;
    if (roundedHoodDeg < minHoodAngle || roundedHoodDeg > maxHoodAngle) {
      return new TurretTuneSetpoint(
          hoodAngleDeg,
          flywheelRps,
          false,
          String.format(
              "Hood %.4f out of range (%.2f-%.2f)", hoodAngleDeg, minHoodAngle, maxHoodAngle));
    }
    hoodAngleDeg = roundedHoodDeg;
    if (!Double.isFinite(flywheelRps)) {
      return new TurretTuneSetpoint(hoodAngleDeg, flywheelRps, false, "Flywheel NaN");
    }
    if (flywheelRps < 0.0) {
      return new TurretTuneSetpoint(
          hoodAngleDeg, flywheelRps, false, "Flywheel must be non-negative");
    }
    return new TurretTuneSetpoint(hoodAngleDeg, flywheelRps, true, "OK");
  }

  private static void publishDefaultTuneValues(Turret turret) {
    SmartDashboard.setDefaultNumber(SHARED_HOOD_KEY, turret.getCommandedHoodAngleDeg());
    SmartDashboard.setDefaultNumber(SHARED_FLYWHEEL_KEY, 0.0);
    SmartDashboard.putBoolean(SHARED_VALID_KEY, true);
    SmartDashboard.putString(SHARED_STATUS_KEY, "Idle");
    SmartDashboard.putNumber(LEFT_TURRET_DISTANCE_KEY, 0.0);
    SmartDashboard.putNumber(RIGHT_TURRET_DISTANCE_KEY, 0.0);
    SmartDashboard.putString(SAVE_STATUS_KEY, "Nothing saved yet");
  }

  private static void applyTurretTune(
      Turret turret, Pose2d robotPose, Translation2d robotToTurret, TurretTuneSetpoint setpoint) {
    turret.runSetpoints(
        getHubHeadingRobot(robotPose, robotToTurret),
        setpoint.hoodAngleDeg(),
        setpoint.flywheelRps());
  }

  static double getDistanceToHub(Pose2d robotPose, Translation2d robotToTurret) {
    return getTurretFieldPosition(robotPose, robotToTurret).getDistance(getHubTranslation());
  }

  private static Rotation2d getHubHeadingRobot(Pose2d robotPose, Translation2d robotToTurret) {
    Rotation2d fieldHeading =
        getHubTranslation().minus(getTurretFieldPosition(robotPose, robotToTurret)).getAngle();
    return fieldHeading.minus(robotPose.getRotation());
  }

  private static Translation2d getTurretFieldPosition(
      Pose2d robotPose, Translation2d robotToTurret) {
    return robotPose.getTranslation().plus(robotToTurret.rotateBy(robotPose.getRotation()));
  }

  record TurretTuneSetpoint(
      double hoodAngleDeg, double flywheelRps, boolean valid, String status) {}
}
