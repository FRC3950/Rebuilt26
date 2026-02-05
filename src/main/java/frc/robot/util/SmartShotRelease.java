package frc.robot.util;

import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.turret.GetAdjustedShot;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class SmartShotRelease {
  private static final String KEY_ENABLED = "Smart Shot/Enabled";
  private static final String KEY_CAN_SHOOT = "Smart Shot/Can Shoot";
  private static final String KEY_TOF = "Smart Shot/Time of Flight";

  static {
    SmartDashboard.setDefaultBoolean(KEY_ENABLED, true);
    SmartDashboard.setDefaultBoolean(KEY_CAN_SHOOT, true);
    SmartDashboard.setDefaultNumber(KEY_TOF, 0.0);
  }

  public static boolean canShoot(double distance) {
    boolean enabled = SmartDashboard.getBoolean(KEY_ENABLED, true);
    Logger.recordOutput(KEY_ENABLED, enabled);
    if (!enabled) return true;

    double timeOfFlight = GetAdjustedShot.getInstance().getTimeOfFlight(distance);

    double matchTime = Timer.getMatchTime();

    // Calculate when the shot will land (Match Time counts DOWN)
    double predictedImpactTime = matchTime - (timeOfFlight + scoreTime);

    SmartDashboard.putNumber(KEY_TOF, timeOfFlight);
    Logger.recordOutput(KEY_TOF, timeOfFlight);
    boolean canShoot = isHubActiveAtTime(predictedImpactTime);
    SmartDashboard.putBoolean(KEY_CAN_SHOOT, canShoot);
    Logger.recordOutput(KEY_CAN_SHOOT, canShoot);
    return canShoot;
  }

  private static boolean isHubActiveAtTime(double time) {
    // Current period info
    boolean isAuto = DriverStation.isAutonomous();
    boolean isTeleop = DriverStation.isTeleop();

    if (isAuto) return true;

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
    if (time <= 30) return true; // End Game

    Optional<Boolean> weWonAuto = AutoWinner.getAutoWinResult();
    if (weWonAuto.isEmpty()) {
      // If we don't know who won, default to allowing shots? Or blocking?
      // Default to allowing to avoid blocking gameplay if FMS fails.
      return true;
    }

    boolean winningAlliance = weWonAuto.get();

    int shift;
    if (time > 105) {
      shift = 1;
    } else if (time > 80) {
      shift = 2;
    } else if (time > 55) {
      shift = 3;
    } else {
      shift = 4;
    }

    boolean winnerActive = (shift % 2 == 0);
    return winnerActive ? winningAlliance : !winningAlliance;
  }
}
