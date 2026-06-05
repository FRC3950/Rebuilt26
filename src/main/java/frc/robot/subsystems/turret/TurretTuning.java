package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TurretTuning {
  static final String FLYWHEEL_SCALE_DASHBOARD_KEY = "Turret/Flywheel Scale";

  private static final double FLYWHEEL_FUDGE_STEP = 0.01;
  private static final double TOF_FUDGE_STEP_SEC = 0.01;
  private static final double TURN_TRIM_STEP_DEG = 0.5;

  private double flywheelScaleFactor = 1.0;
  private double tofFudgeSec = 0.0;
  private double turnTrimDeg = 0.0;

  public void initializeDashboard() {
    SmartDashboard.putNumber(FLYWHEEL_SCALE_DASHBOARD_KEY, flywheelScaleFactor);
  }

  public Command increaseFlywheelFudgeFactor() {
    return Commands.runOnce(
            () ->
                setFlywheelScaleFactor(
                    Math.round(100.0 * getFlywheelScaleFactor() * (1.0 + FLYWHEEL_FUDGE_STEP))
                        / 100.0))
        .ignoringDisable(true)
        .withName("Turret Flywheel Fudge Up");
  }

  public Command decreaseFlywheelFudgeFactor() {
    return Commands.runOnce(
            () ->
                setFlywheelScaleFactor(
                    Math.round(100.0 * getFlywheelScaleFactor() * (1.0 - FLYWHEEL_FUDGE_STEP))
                        / 100.0))
        .ignoringDisable(true)
        .withName("Turret Flywheel Fudge Down");
  }

  public Command increaseTofFudge() {
    return Commands.runOnce(() -> tofFudgeSec += TOF_FUDGE_STEP_SEC)
        .ignoringDisable(true)
        .withName("Turret ToF Fudge Up");
  }

  public Command decreaseTofFudge() {
    return Commands.runOnce(() -> tofFudgeSec -= TOF_FUDGE_STEP_SEC)
        .ignoringDisable(true)
        .withName("Turret ToF Fudge Down");
  }

  public Command increaseTurnTrim() {
    return Commands.runOnce(() -> turnTrimDeg += TURN_TRIM_STEP_DEG)
        .ignoringDisable(true)
        .withName("Turret Turn Trim Left");
  }

  public Command decreaseTurnTrim() {
    return Commands.runOnce(() -> turnTrimDeg -= TURN_TRIM_STEP_DEG)
        .ignoringDisable(true)
        .withName("Turret Turn Trim Right");
  }

  public double getFlywheelScaleFactor() {
    double dashboardValue =
        SmartDashboard.getNumber(FLYWHEEL_SCALE_DASHBOARD_KEY, flywheelScaleFactor);
    flywheelScaleFactor = sanitizeFlywheelScaleFactor(dashboardValue, flywheelScaleFactor);
    if (dashboardValue != flywheelScaleFactor) {
      SmartDashboard.putNumber(FLYWHEEL_SCALE_DASHBOARD_KEY, flywheelScaleFactor);
    }
    return flywheelScaleFactor;
  }

  public double getTofFudgeSec() {
    return tofFudgeSec;
  }

  public double getTurnTrimDeg() {
    return turnTrimDeg;
  }

  private void setFlywheelScaleFactor(double scaleFactor) {
    flywheelScaleFactor = sanitizeFlywheelScaleFactor(scaleFactor, 1.0);
    SmartDashboard.putNumber(FLYWHEEL_SCALE_DASHBOARD_KEY, flywheelScaleFactor);
  }

  private static double sanitizeFlywheelScaleFactor(double scaleFactor, double fallback) {
    return Double.isFinite(scaleFactor) && scaleFactor > 0.0 ? scaleFactor : fallback;
  }
}
