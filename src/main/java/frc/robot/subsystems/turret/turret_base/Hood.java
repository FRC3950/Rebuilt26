package frc.robot.subsystems.turret.turret_base;

import static frc.robot.Constants.SubsystemConstants.Turret.*;

import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import edu.wpi.first.math.MathUtil;
import frc.robot.util.BatteryLogger;
import org.littletonrobotics.junction.Logger;

public class Hood {
  private static final int POWER_STATUS_PERIOD_LOOPS = 5;
  private static ServoHub hoodServoHub;

  private final ServoChannel hoodServo;
  private final boolean invertPulseDirection;
  private double lastSetpointDeg = minHoodAngle;
  private double positionDeg = minHoodAngle;
  private int pulseWidthUs = hoodAngleToPulseWidthUs(minHoodAngle, false);
  private int refreshCounter = 0;

  public Hood(ServoChannel.ChannelId hoodChannelId) {
    hoodServo = getConfiguredHoodServoHub().getServoChannel(hoodChannelId);
    invertPulseDirection = hoodChannelId == HOOD_SERVO_CHANNEL_2;
    initializeAtMinimum();
  }

  public void setAngleDeg(double hoodAngleDeg) {
    double clampedHoodAngleDeg = MathUtil.clamp(hoodAngleDeg, minHoodAngle, maxHoodAngle);
    pulseWidthUs = hoodAngleToPulseWidthUs(clampedHoodAngleDeg, invertPulseDirection);
    hoodServo.setPulseWidth(pulseWidthUs);
    lastSetpointDeg = clampedHoodAngleDeg;
    positionDeg = clampedHoodAngleDeg;
  }

  public void periodic(String logKey) {
    Logger.recordOutput(logKey + "/PositionDeg", positionDeg);
    Logger.recordOutput(logKey + "/PulseWidthUs", pulseWidthUs);
    if (refreshCounter % POWER_STATUS_PERIOD_LOOPS == 0) {
      ServoHub servoHub = getConfiguredHoodServoHub();
      double channelCurrent = hoodServo.getCurrent();
      Logger.recordOutput(logKey + "/HubDeviceVoltageVolts", servoHub.getDeviceVoltage());
      Logger.recordOutput(logKey + "/HubDeviceCurrentAmps", servoHub.getDeviceCurrent());
      Logger.recordOutput(logKey + "/ServoVoltageVolts", servoHub.getServoVoltage());
      Logger.recordOutput(logKey + "/ChannelCurrentAmps", channelCurrent);
      Logger.recordOutput(logKey + "/HubHasActiveFault", servoHub.hasActiveFault());
      Logger.recordOutput(logKey + "/HubHasActiveWarning", servoHub.hasActiveWarning());
      BatteryLogger.reportCurrentUsage(logKey, channelCurrent);
    }
    refreshCounter++;
  }

  public double getPositionDeg() {
    return positionDeg;
  }

  public double getSetpointDeg() {
    return lastSetpointDeg;
  }

  private void initializeAtMinimum() {
    lastSetpointDeg = minHoodAngle;
    positionDeg = minHoodAngle;
    pulseWidthUs = hoodAngleToPulseWidthUs(minHoodAngle, invertPulseDirection);

    hoodServo.setEnabled(true);
    hoodServo.setPowered(true);
    hoodServo.setPulseWidth(pulseWidthUs);
  }

  private static synchronized ServoHub getConfiguredHoodServoHub() {
    if (hoodServoHub == null) {
      hoodServoHub = new ServoHub(HOOD_SERVO_HUB_CAN_ID);

      ServoHubConfig hubConfig = new ServoHubConfig();
      ServoChannelConfig turret1Config =
          new ServoChannelConfig(HOOD_SERVO_CHANNEL_1)
              .pulseRange(
                  HOOD_SERVO_MIN_PULSE_US, HOOD_SERVO_CENTER_PULSE_US, HOOD_SERVO_MAX_PULSE_US)
              .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
      ServoChannelConfig turret2Config =
          new ServoChannelConfig(HOOD_SERVO_CHANNEL_2)
              .pulseRange(
                  HOOD_SERVO_MIN_PULSE_US, HOOD_SERVO_CENTER_PULSE_US, HOOD_SERVO_MAX_PULSE_US)
              .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);

      hubConfig.apply(HOOD_SERVO_CHANNEL_1, turret1Config);
      hubConfig.apply(HOOD_SERVO_CHANNEL_2, turret2Config);
      hoodServoHub.configure(hubConfig, ResetMode.kNoResetSafeParameters);
    }
    return hoodServoHub;
  }

  private static int hoodAngleToPulseWidthUs(double hoodAngleDeg, boolean invertPulseDirection) {
    double clampedHoodAngleDeg = MathUtil.clamp(hoodAngleDeg, minHoodAngle, maxHoodAngle);
    double hoodAngleRangeDeg = maxHoodAngle - minHoodAngle;
    int servoPulseRangeUs = HOOD_SERVO_MAX_PULSE_US - HOOD_SERVO_MIN_PULSE_US;
    if (hoodAngleRangeDeg <= 0.0 || servoPulseRangeUs <= 0) {
      return HOOD_SERVO_MIN_PULSE_US;
    }

    double t = (clampedHoodAngleDeg - minHoodAngle) / hoodAngleRangeDeg;
    if (invertPulseDirection) {
      t = 1.0 - t;
    }
    int pulseUs = (int) Math.round(HOOD_SERVO_MIN_PULSE_US + t * servoPulseRangeUs);
    return Math.max(HOOD_SERVO_MIN_PULSE_US, Math.min(HOOD_SERVO_MAX_PULSE_US, pulseUs));
  }
}
