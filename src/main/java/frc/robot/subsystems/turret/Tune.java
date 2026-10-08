package frc.robot.subsystems.turret;

import static frc.robot.Constants.FieldConstants.getHubTranslation;
import static frc.robot.Constants.SubsystemConstants.Turret.maxHoodAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.minHoodAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret1;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Distancer;
import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

/**
 * Shot table tuning. Both turrets share one hood/flywheel setpoint because they read the same
 * shot_table.json. Line up head-on to the hub; saved rows use the mean of the two turret distances.
 */
public final class Tune extends SubsystemBase {
  private static final String SHARED_HOOD_KEY = "Tune/Hood Deg";
  private static final String SHARED_FLYWHEEL_KEY = "Tune/Flywheel RPS";
  private static final String SHARED_VALID_KEY = "Tune/Setpoints Valid";
  private static final String SHARED_STATUS_KEY = "Tune/Setpoints Status";
  private static final String LEFT_TURRET_DISTANCE_KEY = "Tune/Left Turret To Hub Distance M";
  private static final String RIGHT_TURRET_DISTANCE_KEY = "Tune/Right Turret To Hub Distance M";
  private static final String SAVE_STATUS_KEY = "Tune/Save Status";
  private static final String TABLE_HOOD_KEY = "Tune/Table Hood Deg";
  private static final String TABLE_FLYWHEEL_KEY = "Tune/Table Flywheel RPS";
  // Commanded, not measured: the hood servos have no feedback.
  private static final String LEFT_TURRET_HOOD_KEY = "Tune/Left Turret Commanded Hood Deg";
  private static final String RIGHT_TURRET_HOOD_KEY = "Tune/Right Turret Commanded Hood Deg";
  private static final String LEFT_TURRET_COMMANDED_RPS_KEY = "Tune/Left Turret Commanded RPS";
  private static final String RIGHT_TURRET_COMMANDED_RPS_KEY = "Tune/Right Turret Commanded RPS";
  private static final String LEFT_TURRET_MEASURED_RPS_KEY = "Tune/Left Turret Measured RPS";
  private static final String RIGHT_TURRET_MEASURED_RPS_KEY = "Tune/Right Turret Measured RPS";

  private static final String USE_TUNE_VALUES_KEY = "Tune/Use Tune Values";

  private final Supplier<Pose2d> robotPose;
  private final Turret leftTurret;
  private final Turret rightTurret;

  public Tune(Supplier<Pose2d> robotPose, Turret leftTurret, Turret rightTurret) {
    this.robotPose = robotPose;
    this.leftTurret = leftTurret;
    this.rightTurret = rightTurret;
    initializeDashboard(robotPose, leftTurret.getCommandedHoodAngleDeg());
    publishTuneTelemetry(robotPose.get(), leftTurret, rightTurret);
  }

  @Override
  public void periodic() {
    publishTuneTelemetry(robotPose.get(), leftTurret, rightTurret);
  }

  /** Keep controls available in every code mode; never resume tuning automatically at startup. */
  static void initializeDashboard(Supplier<Pose2d> robotPose, double currentHoodDeg) {
    SmartDashboard.putBoolean(USE_TUNE_VALUES_KEY, false);
    publishDefaultTuneValues(currentHoodDeg);
    SmartDashboard.putData(
        "Tune/Save Point",
        Commands.runOnce(() -> saveTunePoint(robotPose.get())).ignoringDisable(true));
  }

  public static boolean useTuneValues() {
    return SmartDashboard.getBoolean(USE_TUNE_VALUES_KEY, false);
  }

  /** Called by the targeting command that owns this turret, including ferry targeting. */
  public static void runTuneValues(Turret turret, Pose2d robotPose, Translation2d robotToTurret) {
    TurretTuneSetpoint setpoint = readTuneSetpoint();
    if (!setpoint.valid()) {
      turret.stop();
      return;
    }
    applyTurretTune(turret, robotPose, robotToTurret, setpoint);
  }

  private static TurretTuneSetpoint readTuneSetpoint() {
    return validateTuneSetpoint(
        SmartDashboard.getNumber(SHARED_HOOD_KEY, Double.NaN),
        SmartDashboard.getNumber(SHARED_FLYWHEEL_KEY, 0.0));
  }

  private static void saveTunePoint(Pose2d robotPose) {
    TurretTuneSetpoint setpoint = readTuneSetpoint();
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
    double distanceMeters = getTuneDistance(robotPose);
    String result;
    try {
      File file =
          GetAdjustedShot.saveTunedRow(
              distanceMeters, setpoint.hoodAngleDeg(), setpoint.flywheelRps());
      result = "Saved to " + file.getPath();
    } catch (IOException e) {
      result = "In use but FILE WRITE FAILED (" + e.getMessage() + ")";
    }

    SmartDashboard.putString(
        SAVE_STATUS_KEY,
        String.format(
            "%s d=%.2f m, hood=%.1f, rps=%.1f",
            result, distanceMeters, setpoint.hoodAngleDeg(), setpoint.flywheelRps()));
  }

  private static void publishTuneTelemetry(
      Pose2d robotPose, Turret leftTurret, Turret rightTurret) {
    TurretTuneSetpoint setpoint = readTuneSetpoint();
    SmartDashboard.putBoolean(SHARED_VALID_KEY, setpoint.valid());
    SmartDashboard.putString(SHARED_STATUS_KEY, setpoint.status());
    SmartDashboard.putNumber(LEFT_TURRET_DISTANCE_KEY, getDistanceToHub(robotPose, robotToTurret1));
    SmartDashboard.putNumber(
        RIGHT_TURRET_DISTANCE_KEY, getDistanceToHub(robotPose, robotToTurret2));
    SmartDashboard.putNumber(LEFT_TURRET_HOOD_KEY, leftTurret.getCommandedHoodAngleDeg());
    SmartDashboard.putNumber(RIGHT_TURRET_HOOD_KEY, rightTurret.getCommandedHoodAngleDeg());
    SmartDashboard.putNumber(LEFT_TURRET_COMMANDED_RPS_KEY, leftTurret.getCommandedFlywheelRps());
    SmartDashboard.putNumber(RIGHT_TURRET_COMMANDED_RPS_KEY, rightTurret.getCommandedFlywheelRps());
    SmartDashboard.putNumber(
        LEFT_TURRET_MEASURED_RPS_KEY, roundToHundredths(leftTurret.getMeasuredFlywheelRps()));
    SmartDashboard.putNumber(
        RIGHT_TURRET_MEASURED_RPS_KEY, roundToHundredths(rightTurret.getMeasuredFlywheelRps()));

    // What the shot table currently says for this spot, at the same distance a save would use.
    Distancer tableShot = GetAdjustedShot.getTableShot(getTuneDistance(robotPose));
    // Clamped like the turret does, so the value can be copied straight into Tune/Hood Deg.
    SmartDashboard.putNumber(
        TABLE_HOOD_KEY,
        tableShot != null
            ? roundToHundredths(
                MathUtil.clamp(tableShot.hoodAngleDeg(), minHoodAngle, maxHoodAngle))
            : 0.0);
    SmartDashboard.putNumber(
        TABLE_FLYWHEEL_KEY, tableShot != null ? roundToHundredths(tableShot.flywheelRps()) : 0.0);
  }

  private static double getTuneDistance(Pose2d robotPose) {
    return (getDistanceToHub(robotPose, robotToTurret1)
            + getDistanceToHub(robotPose, robotToTurret2))
        / 2.0;
  }

  private static double roundToHundredths(double value) {
    return Math.round(value * 100.0) / 100.0;
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

  private static void publishDefaultTuneValues(double hoodDeg) {
    SmartDashboard.setDefaultNumber(SHARED_HOOD_KEY, hoodDeg);
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
