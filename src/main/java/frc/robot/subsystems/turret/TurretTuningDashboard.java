package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class TurretTuningDashboard {
  private TurretTuningDashboard() {}

  public static void register(Turret leftTurret, Turret rightTurret) {
    registerTurret("Left", leftTurret);
    registerTurret("Right", rightTurret);
  }

  private static void registerTurret(String side, Turret turret) {
    String keyPrefix = "Turret/" + side + "/";
    SmartDashboard.putData(keyPrefix + "Flywheel Fudge Up", turret.increaseFlywheelFudgeFactor());
    SmartDashboard.putData(keyPrefix + "Flywheel Fudge Down", turret.decreaseFlywheelFudgeFactor());
    SmartDashboard.putData(keyPrefix + "ToF Fudge Up", turret.increaseTofFudge());
    SmartDashboard.putData(keyPrefix + "ToF Fudge Down", turret.decreaseTofFudge());
    SmartDashboard.putData(keyPrefix + "Turn Trim Left", turret.increaseTurnTrim());
    SmartDashboard.putData(keyPrefix + "Turn Trim Right", turret.decreaseTurnTrim());
  }
}
