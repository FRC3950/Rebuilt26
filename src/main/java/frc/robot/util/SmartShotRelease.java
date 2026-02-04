package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.turret.GetAdjustedShot;
import static frc.robot.Constants.FieldConstants.*;
import java.util.Optional;

public class SmartShotRelease {
  private static final String KEY_ENABLED = "Smart Shot/Enabled";
  private static final String KEY_CAN_SHOOT = "Smart Shot/Can Shoot";
  private static final String KEY_TOF = "Smart Shot/Time of Flight";

  static {
    SmartDashboard.setDefaultBoolean(KEY_ENABLED, true);
    SmartDashboard.setDefaultBoolean(KEY_CAN_SHOOT, true);
    SmartDashboard.setDefaultNumber(KEY_TOF, 0.0);  }

  public static boolean canShoot(double distance) {
    boolean enabled = SmartDashboard.getBoolean(KEY_ENABLED, true);
    if (!enabled) return true;

    double timeOfFlight = GetAdjustedShot.getInstance().getTimeOfFlight(distance);
    
    double matchTime = Timer.getMatchTime();

    // Calculate when the shot will land (Match Time counts DOWN)
    double predictedImpactTime = matchTime - (timeOfFlight + scoreTime);

    SmartDashboard.putNumber(KEY_TOF, timeOfFlight);
    boolean canShoot = isHubActiveAtTime(predictedImpactTime);
    SmartDashboard.putBoolean(KEY_CAN_SHOOT, canShoot);

    return canShoot;
  }

  private static boolean isHubActiveAtTime(double time) {
    // Current period info
    boolean isAuto = DriverStation.isAutonomous();
    boolean isTeleop = DriverStation.isTeleop();

    if (isAuto) {
      // Auto is always active.
      // Note: Timer.getMatchTime() in Auto counts down from 15 (or 20 depending on
      // year configs/sim).
      // The prompt says Auto is 20s (0:20 - 0:00).
      // If predicted time < 0, it means it lands in Teleop transition or Teleop.
      if (time >= 0) return true;

      // If it crosses into teleop, we need to handle that.
      // However, FRC match timer behavior across boundaries can be tricky.
      // Usually getMatchTime resets or switches.
      // Let's assume for simplicity: If currently in Auto, and shot lands in Auto,
      // OK.
      // If currently in Auto, but shot lands "after" (negative time?),
      // usually there is a disabled gap or it counts as Teleop.
      // Given the 3 second delay mentioned in rules, likely shots late in auto don't
      // score or count.
      // Rule says: "Both ALLIANCE HUBS are active during AUTO".
      // If it lands in the gap, it might count?
      // Safe bet: If currently Auto, return true.
      return true;
    }

    if (!isTeleop) return false; // Disabled or Test

    // Teleop Logic
    // Time starts at 140 (2:20) and counts down to 0.
    // Transition: 140 - 130 (Active)
    // Shift 1: 130 - 105
    // Shift 2: 105 - 80
    // Shift 3: 80 - 55
    // Shift 4: 55 - 30
    // End Game: 30 - 0 (Active)

    if (time > 130) return true; // Transition
    if (time < 30) return true; // End Game

    Optional<Boolean> weWonAuto = AutoWinner.getAutoWinResult();
    if (weWonAuto.isEmpty()) {
      // If we don't know who won, default to allowing shots? Or blocking?
      // Default to allowing to avoid blocking gameplay if FMS fails.
      return true;
    }

    boolean winningAlliance = weWonAuto.get();

    // Shift 1 (130-105): Winner Inactive / Loser Active
    if (time <= 130 && time > 105) {
      return !winningAlliance;
    }

    // Shift 2 (105-80): Winner Active / Loser Inactive
    if (time <= 105 && time > 80) {
      return winningAlliance;
    }

    // Shift 3 (80-55): Winner Inactive / Loser Active
    if (time <= 80 && time > 55) {
      return !winningAlliance;
    }

    // Shift 4 (55-30): Winner Active / Loser Inactive
    if (time <= 55 && time > 30) {
      return winningAlliance;
    }

    return true; // Should be covered by ranges, but fallback true.
  }
}
