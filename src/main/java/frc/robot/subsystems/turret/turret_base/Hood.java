package frc.robot.subsystems.turret.turret_base;

import static frc.robot.Constants.SubsystemConstants.Turret.*;

import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import edu.wpi.first.math.MathUtil;
import java.util.function.Function;

public class Hood {
  private static ServoHub hoodServoHub;

  private final ServoChannel hoodServo;
  private final EnableState enableState;
  private final boolean invertPulseDirection;
  private double lastSetpointDeg = minHoodAngle;
  private double positionDeg = minHoodAngle;

  public Hood(ServoChannel.ChannelId hoodChannelId) {
    hoodServo = getConfiguredHoodServoHub().getServoChannel(hoodChannelId);
    enableState = new EnableState(hoodServo::setEnabled);
    invertPulseDirection = hoodChannelId == HOOD_SERVO_CHANNEL_2;
    initializeAtMinimum();
  }

  public void setAngleDeg(double hoodAngleDeg) {
    double clampedHoodAngleDeg = MathUtil.clamp(hoodAngleDeg, minHoodAngle, maxHoodAngle);
    hoodServo.setPulseWidth(hoodAngleToPulseWidthUs(clampedHoodAngleDeg, invertPulseDirection));
    enableState.setEnabled(true);
    lastSetpointDeg = clampedHoodAngleDeg;
    positionDeg = clampedHoodAngleDeg;
  }

  public double getPositionDeg() {
    return positionDeg;
  }

  public double getSetpointDeg() {
    return lastSetpointDeg;
  }

  public void stop() {
    // Disable position pulses; retain the configured power behavior while disabled.
    enableState.setEnabled(false);
  }

  private void initializeAtMinimum() {
    lastSetpointDeg = minHoodAngle;
    positionDeg = minHoodAngle;

    enableState.setEnabled(true);
    hoodServo.setPowered(true);
    hoodServo.setPulseWidth(hoodAngleToPulseWidthUs(minHoodAngle, invertPulseDirection));
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

  /** Skips redundant successful writes; a failed write leaves the state unknown. */
  static final class EnableState {
    private final Function<Boolean, REVLibError> writeEnabled;
    private Boolean lastSuccessfulEnabled;

    EnableState(Function<Boolean, REVLibError> writeEnabled) {
      this.writeEnabled = writeEnabled;
    }

    void setEnabled(boolean enabled) {
      if (lastSuccessfulEnabled != null && lastSuccessfulEnabled == enabled) {
        return;
      }

      lastSuccessfulEnabled = writeEnabled.apply(enabled) == REVLibError.kOk ? enabled : null;
    }
  }
}
