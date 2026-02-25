package frc.robot.subsystems.turret.turret_base;

import static frc.robot.Constants.SubsystemConstants.Turret.*;

import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import edu.wpi.first.math.MathUtil;

public class Hood {
  private static ServoHub hoodServoHub;

  private final ServoChannel hoodServo;
  private double lastSetpointDeg = minHoodAngle;
  private double positionDeg = minHoodAngle;

  public Hood(ServoChannel.ChannelId hoodChannelId) {
    hoodServo = getConfiguredHoodServoHub().getServoChannel(hoodChannelId);
    initializeAtMinimum();
  }

  public void setAngleDeg(double hoodAngleDeg) {
    double clampedHoodAngleDeg = MathUtil.clamp(hoodAngleDeg, minHoodAngle, maxHoodAngle);
    hoodServo.setPulseWidth(hoodAngleToPulseWidthUs(clampedHoodAngleDeg));
    lastSetpointDeg = clampedHoodAngleDeg;
  }

  public void hoodDown() {
    setAngleDeg(minHoodAngle);
  }

  public void periodic() {
    double measuredHoodAngleDeg = pulseWidthUsToHoodAngleDeg(hoodServo.getPulseWidth());
    if (!Double.isFinite(measuredHoodAngleDeg)) {
      measuredHoodAngleDeg = lastSetpointDeg;
    }

    positionDeg = measuredHoodAngleDeg;
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

    hoodServo.setEnabled(true);
    hoodServo.setPowered(true);
    hoodServo.setPulseWidth(HOOD_SERVO_MIN_PULSE_US);
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

  private static int hoodAngleToPulseWidthUs(double hoodAngleDeg) {
    double clampedHoodAngleDeg = MathUtil.clamp(hoodAngleDeg, minHoodAngle, maxHoodAngle);
    double hoodAngleRangeDeg = maxHoodAngle - minHoodAngle;
    int servoPulseRangeUs = HOOD_SERVO_MAX_PULSE_US - HOOD_SERVO_MIN_PULSE_US;
    if (hoodAngleRangeDeg <= 0.0 || servoPulseRangeUs <= 0) {
      return HOOD_SERVO_MIN_PULSE_US;
    }

    double t = (clampedHoodAngleDeg - minHoodAngle) / hoodAngleRangeDeg;
    int pulseUs = (int) Math.round(HOOD_SERVO_MIN_PULSE_US + t * servoPulseRangeUs);
    return Math.max(HOOD_SERVO_MIN_PULSE_US, Math.min(HOOD_SERVO_MAX_PULSE_US, pulseUs));
  }

  private static double pulseWidthUsToHoodAngleDeg(int pulseWidthUs) {
    double hoodAngleRangeDeg = maxHoodAngle - minHoodAngle;
    int servoPulseRangeUs = HOOD_SERVO_MAX_PULSE_US - HOOD_SERVO_MIN_PULSE_US;
    if (hoodAngleRangeDeg <= 0.0 || servoPulseRangeUs <= 0) {
      return minHoodAngle;
    }

    int clampedPulseUs =
        Math.max(HOOD_SERVO_MIN_PULSE_US, Math.min(HOOD_SERVO_MAX_PULSE_US, pulseWidthUs));
    double t = (double) (clampedPulseUs - HOOD_SERVO_MIN_PULSE_US) / servoPulseRangeUs;
    return minHoodAngle + t * hoodAngleRangeDeg;
  }
}
