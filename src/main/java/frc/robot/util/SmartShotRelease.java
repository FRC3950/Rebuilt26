package frc.robot.util;

import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.turret.GetAdjustedShot;
import org.littletonrobotics.junction.Logger;

public class SmartShotRelease {
  private static final String KEY_ENABLED = "Smart Shot/Enabled";
  private static final String KEY_CAN_SHOOT = "Smart Shot/Can Shoot";
  private static final String KEY_TOF = "Smart Shot/Time of Flight";
  private static final String KEY_PREDICTED_IMPACT_DELAY = "Smart Shot/Predicted Impact Delay";
  private static final String KEY_SHIFT = "Smart Shot/Current Shift";
  private static final String KEY_SHIFT_ACTIVE = "Smart Shot/Shift Active";
  private static final String KEY_SHIFT_REMAINING = "Smart Shot/Shift Remaining Time";

  static {
    SmartDashboard.setDefaultBoolean(KEY_ENABLED, true);
    SmartDashboard.setDefaultBoolean(KEY_CAN_SHOOT, true);
    SmartDashboard.setDefaultNumber(KEY_TOF, 0.0);
    SmartDashboard.setDefaultNumber(KEY_PREDICTED_IMPACT_DELAY, 0.0);
  }

  public static boolean canShoot(double distance) {
    HubShiftUtil.ensureMatchTimerStarted();

    boolean enabled = SmartDashboard.getBoolean(KEY_ENABLED, true);
    Logger.recordOutput(KEY_ENABLED, enabled);
    if (!enabled) return true;

    double timeOfFlight = GetAdjustedShot.getInstance().getTimeOfFlight(distance);
    double predictedImpactDelaySec = timeOfFlight + scoreTime;

    SmartDashboard.putNumber(KEY_TOF, timeOfFlight);
    SmartDashboard.putNumber(KEY_PREDICTED_IMPACT_DELAY, predictedImpactDelaySec);
    Logger.recordOutput(KEY_TOF, timeOfFlight);

    HubShiftUtil.ShiftInfo shiftInfo = HubShiftUtil.getShiftedShiftInfo();
    SmartDashboard.putString(KEY_SHIFT, shiftInfo.currentShift().name());
    SmartDashboard.putBoolean(KEY_SHIFT_ACTIVE, shiftInfo.active());
    SmartDashboard.putNumber(KEY_SHIFT_REMAINING, shiftInfo.remainingTimeSec());
    Logger.recordOutput(KEY_SHIFT, shiftInfo.currentShift().name());
    Logger.recordOutput(KEY_SHIFT_ACTIVE, shiftInfo.active());
    Logger.recordOutput(KEY_SHIFT_REMAINING, shiftInfo.remainingTimeSec());

    // Shifted shift windows already include launch/processing timing offsets.
    boolean canShoot = HubShiftUtil.isHubActiveAtPredictedImpact(0.0);
    SmartDashboard.putBoolean(KEY_CAN_SHOOT, canShoot);
    Logger.recordOutput(KEY_CAN_SHOOT, canShoot);
    return canShoot;
  }
}
