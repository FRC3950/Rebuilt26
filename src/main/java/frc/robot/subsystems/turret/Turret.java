package frc.robot.subsystems.turret;

import static frc.robot.Constants.SubsystemConstants.Turret.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.turret_base.azimuth.Azimuth;
import frc.robot.subsystems.turret.turret_base.flywheels.Flywheels;
import frc.robot.subsystems.turret.turret_base.hood.Hood;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private static final double ANGLE_WRAP_DEGREES = 360.0;
  private static final double TURRET_SETPOINT_LOOKAHEAD_SECS = 0.02;
  private static final double FLYWHEEL_READY_TOLERANCE_RPS = 2.5;
  private static final double FLYWHEEL_FUDGE_STEP = 0.01;
  private static final double TOF_FUDGE_STEP_SEC = 0.01;
  private static final double TURN_TRIM_STEP_DEG = 0.5;

  private final Hood hood;
  private final Flywheels flywheels;
  private final Azimuth azimuth;
  private final String logKey;
  private final double minAzimuthControlAngleDeg;
  private final double maxAzimuthControlAngleDeg;
  private static boolean lockedIn = false;
  private boolean zeroSwitchClosedLastPoll = false;
  private double flywheelFudgeFactor = 1.0;
  private double tofFudgeSec = 0.0;
  private double turnTrimDeg = 0.0;

  // private final Mechanism2d mechanism;
  // private final MechanismRoot2d mechRoot;
  // private final MechanismLigament2d turretLigament;

  public Turret(
      String name,
      Azimuth azimuth,
      Hood hood,
      Flywheels flywheels,
      double minAzimuthControlAngleDeg,
      double maxAzimuthControlAngleDeg) {
    setName(name);
    this.azimuth = azimuth;
    this.hood = hood;
    this.flywheels = flywheels;
    logKey = createLogKey(name);
    this.minAzimuthControlAngleDeg = minAzimuthControlAngleDeg;
    this.maxAzimuthControlAngleDeg = maxAzimuthControlAngleDeg;
  }

  String getLogKey() {
    return logKey;
  }

  /** Returns the live turret pose with robot-relative translation and measured azimuth. */
  public Pose3d getRobotPose3d(Translation2d robotToTurret) {
    return new Pose3d(
        robotToTurret.getX(),
        robotToTurret.getY(),
        0.325,
        new Rotation3d(0.0, 0.0, Units.degreesToRadians(getVisualizationAngleDeg())));
  }

  private double getVisualizationAngleDeg() {
    return Constants.currentMode == Constants.Mode.SIM
        ? azimuth.getSetpointDeg()
        : azimuth.getMeasuredAngleDeg();
  }

  public void runSetpoints(Rotation2d turretAngleRobot, double hoodAngleDeg, double flywheelSpeed) {
    runSetpoints(turretAngleRobot, 0.0, hoodAngleDeg, flywheelSpeed);
  }

  private void runSetpoints(
      Rotation2d turretAngleRobot,
      double turretVelocityRadPerSec,
      double hoodAngleDeg,
      double flywheelSpeed) {
    double turretVelocityDegPerSec = Units.radiansToDegrees(turretVelocityRadPerSec);
    double targetAzimuthDegrees =
        turretAngleRobot.getDegrees()
            + turnTrimDeg
            + turretVelocityDegPerSec * TURRET_SETPOINT_LOOKAHEAD_SECS;
    double setpointDegrees = selectSafeSetpointDegrees(targetAzimuthDegrees);
    double clampedHoodAngleDeg = MathUtil.clamp(hoodAngleDeg, minHoodAngle, maxHoodAngle);

    azimuth.setTargetAngleDeg(setpointDegrees, turretVelocityDegPerSec);
    hood.setAngleDeg(clampedHoodAngleDeg);
    flywheels.setTargetRps(flywheelSpeed);
  }

  public void runAutoTarget(GetAdjustedShot.ShootingParameters params) {
    runSetpoints(
        params.turretAngle(),
        params.turretVelocity(),
        params.hoodAngleDeg(),
        params.flywheelSpeed() * flywheelFudgeFactor);
  }

  public void runZeroAzimuthTarget(GetAdjustedShot.ShootingParameters params) {
    runSetpoints(
        Rotation2d.fromDegrees(-135),
        0.0,
        params.hoodAngleDeg(),
        params.flywheelSpeed() * flywheelFudgeFactor);
  }

  public Command increaseFlywheelFudgeFactor() {
    return Commands.runOnce(
            () ->
                flywheelFudgeFactor =
                    Math.round(100.0 * flywheelFudgeFactor * (1.0 + FLYWHEEL_FUDGE_STEP)) / 100.0)
        .ignoringDisable(true)
        .withName(getName() + " Flywheel Fudge Up");
  }

  public Command decreaseFlywheelFudgeFactor() {
    return Commands.runOnce(
            () ->
                flywheelFudgeFactor =
                    Math.round(100.0 * flywheelFudgeFactor * (1.0 - FLYWHEEL_FUDGE_STEP)) / 100.0)
        .ignoringDisable(true)
        .withName(getName() + " Flywheel Fudge Down");
  }

  public Command increaseTofFudge() {
    return Commands.runOnce(() -> tofFudgeSec += TOF_FUDGE_STEP_SEC)
        .ignoringDisable(true)
        .withName(getName() + " ToF Fudge Up");
  }

  public Command decreaseTofFudge() {
    return Commands.runOnce(() -> tofFudgeSec -= TOF_FUDGE_STEP_SEC)
        .ignoringDisable(true)
        .withName(getName() + " ToF Fudge Down");
  }

  public Command increaseTurnTrim() {
    return Commands.runOnce(() -> turnTrimDeg += TURN_TRIM_STEP_DEG)
        .ignoringDisable(true)
        .withName(getName() + " Turn Trim Left");
  }

  public Command decreaseTurnTrim() {
    return Commands.runOnce(() -> turnTrimDeg -= TURN_TRIM_STEP_DEG)
        .ignoringDisable(true)
        .withName(getName() + " Turn Trim Right");
  }

  public static void toggleTurretMode() {
    if (lockedIn) {
      lockedIn = false;
    } else {
      lockedIn = true;
    }
  }

  public static boolean getTargetingMode() {
    return lockedIn;
  }

  public double getCommandedAzimuthDeg() {
    return azimuth.getSetpointDeg();
  }

  public double getCommandedHoodAngleDeg() {
    return hood.getSetpointDeg();
  }

  public double getCommandedFlywheelRps() {
    return flywheels.getTargetRps();
  }

  public double getMeasuredAzimuthDeg() {
    return azimuth.getMeasuredAngleDeg();
  }

  public double getRequestedHoodAngleDeg() {
    return hood.getSetpointDeg();
  }

  public double getMeasuredFlywheelRps() {
    return flywheels.getMeasuredVelocityRps();
  }

  public double getMeasuredFlywheelFollowerRps() {
    return flywheels.getFollowerVelocityRps();
  }

  public double getFlywheelFudgeFactor() {
    return flywheelFudgeFactor;
  }

  public double getTofFudgeSec() {
    return tofFudgeSec;
  }

  public double getTurnTrimDeg() {
    return turnTrimDeg;
  }

  public double getCommandedAzimuthVelocityDegPerSec() {
    return azimuth.getVelocitySetpointDegPerSec();
  }

  public boolean isTargetingLocked() {
    return getTargetingMode();
  }

  public boolean isFlywheelReadyForFeed() {
    return flywheels.isReadyForFeed(FLYWHEEL_READY_TOLERANCE_RPS);
  }

  @Override
  public void periodic() {
    azimuth.periodic();
    hood.periodic();
    flywheels.periodic();

    if (DriverStation.isDisabled()) {
      updateDisabledZeroing();
    } else {
      zeroSwitchClosedLastPoll = false;
    }

    logOutputs();
  }

  private void updateDisabledZeroing() {
    boolean zeroSwitchClosed = azimuth.isZeroSwitchClosed();
    if (zeroSwitchClosed && !zeroSwitchClosedLastPoll) {
      azimuth.zeroPosition();
    }
    zeroSwitchClosedLastPoll = zeroSwitchClosed;
  }

  private void logOutputs() {
    Logger.recordOutput(logKey + "/CommandedAzimuthDeg", getCommandedAzimuthDeg());
    Logger.recordOutput(
        logKey + "/CommandedAzimuthVelocityDegPerSec", getCommandedAzimuthVelocityDegPerSec());
    Logger.recordOutput(logKey + "/CommandedHoodAngleDeg", getCommandedHoodAngleDeg());
    Logger.recordOutput(logKey + "/RequestedHoodAngleDeg", getRequestedHoodAngleDeg());
    Logger.recordOutput(logKey + "/CommandedFlywheelRps", getCommandedFlywheelRps());
    Logger.recordOutput(logKey + "/MeasuredAzimuthDeg", getMeasuredAzimuthDeg());
    Logger.recordOutput(logKey + "/MeasuredFlywheelRps", getMeasuredFlywheelRps());
    Logger.recordOutput(logKey + "/MeasuredFlywheelFollowerRps", getMeasuredFlywheelFollowerRps());
    Logger.recordOutput(logKey + "/FlywheelFudgeFactor", getFlywheelFudgeFactor());
    Logger.recordOutput(logKey + "/TofFudgeSec", getTofFudgeSec());
    Logger.recordOutput(logKey + "/TurnTrimDeg", getTurnTrimDeg());
    Logger.recordOutput(logKey + "/IsTargetingLocked", isTargetingLocked());
    Logger.recordOutput(logKey + "/IsFlywheelReadyForFeed", isFlywheelReadyForFeed());
  }

  private static String createLogKey(String name) {
    if (name.endsWith("Turret") && name.length() > "Turret".length()) {
      return "Turret/" + name.substring(0, name.length() - "Turret".length());
    }
    return "Turret/" + name;
  }

  private double selectSafeSetpointDegrees(double targetAzimuthDegrees) {
    double referenceSetpointDegrees =
        MathUtil.clamp(
            azimuth.getSetpointDeg(), minAzimuthControlAngleDeg, maxAzimuthControlAngleDeg);
    double bestCandidateDegrees = Double.NaN;
    double bestErrorDegrees = Double.POSITIVE_INFINITY;

    int minWrapIndex =
        (int) Math.ceil((minAzimuthControlAngleDeg - targetAzimuthDegrees) / ANGLE_WRAP_DEGREES);
    int maxWrapIndex =
        (int) Math.floor((maxAzimuthControlAngleDeg - targetAzimuthDegrees) / ANGLE_WRAP_DEGREES);

    for (int wrapIndex = minWrapIndex; wrapIndex <= maxWrapIndex; wrapIndex++) {
      double candidateDegrees = targetAzimuthDegrees + ANGLE_WRAP_DEGREES * wrapIndex;
      double errorDegrees = Math.abs(candidateDegrees - referenceSetpointDegrees);
      if (errorDegrees < bestErrorDegrees) {
        bestCandidateDegrees = candidateDegrees;
        bestErrorDegrees = errorDegrees;
      }
    }

    if (Double.isNaN(bestCandidateDegrees)) {
      return MathUtil.clamp(
          targetAzimuthDegrees, minAzimuthControlAngleDeg, maxAzimuthControlAngleDeg);
    }
    return bestCandidateDegrees;
  }
}
