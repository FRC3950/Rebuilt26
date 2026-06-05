package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class TurretTuningDashboard {
  private TurretTuningDashboard() {}

  public static void register(TurretTuning tuning) {
    tuning.initializeDashboard();
    SmartDashboard.putData("Turret/Flywheel Scale Up", tuning.increaseFlywheelFudgeFactor());
    SmartDashboard.putData("Turret/Flywheel Scale Down", tuning.decreaseFlywheelFudgeFactor());
    SmartDashboard.putData("Turret/ToF Fudge Up", tuning.increaseTofFudge());
    SmartDashboard.putData("Turret/ToF Fudge Down", tuning.decreaseTofFudge());
    SmartDashboard.putData("Turret/Turn Trim Left", tuning.increaseTurnTrim());
    SmartDashboard.putData("Turret/Turn Trim Right", tuning.decreaseTurnTrim());
  }
}
