package frc.robot.subsystems.turret.turret_base.hood;

import static frc.robot.Constants.SubsystemConstants.Turret.HOOD_SERVO_CENTER_PULSE_US;
import static frc.robot.Constants.SubsystemConstants.Turret.HOOD_SERVO_CHANNEL_1;
import static frc.robot.Constants.SubsystemConstants.Turret.HOOD_SERVO_CHANNEL_2;
import static frc.robot.Constants.SubsystemConstants.Turret.HOOD_SERVO_HUB_CAN_ID;
import static frc.robot.Constants.SubsystemConstants.Turret.HOOD_SERVO_MAX_PULSE_US;
import static frc.robot.Constants.SubsystemConstants.Turret.HOOD_SERVO_MIN_PULSE_US;
import static frc.robot.Constants.SubsystemConstants.Turret.maxHoodAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.minHoodAngle;

import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import edu.wpi.first.math.MathUtil;

public class HoodIOServoHub implements HoodIO {
  private static final int POWER_STATUS_PERIOD_LOOPS = 5;

  private static ServoHub hoodServoHub;
  private static boolean hoodServoHubConfigured = false;

  private final ServoChannel hoodServo;
  private final boolean invertPulseDirection;
  private boolean lastCommandSucceeded = true;
  private double positionDeg = minHoodAngle;
  private int pulseWidthUs = hoodAngleToPulseWidthUs(minHoodAngle, false);
  private int refreshCounter = 0;

  public HoodIOServoHub(ServoChannel.ChannelId hoodChannelId) {
    hoodServo = getConfiguredHoodServoHub().getServoChannel(hoodChannelId);
    invertPulseDirection = hoodChannelId == HOOD_SERVO_CHANNEL_2;
    setAngleDeg(minHoodAngle);
    lastCommandSucceeded =
        lastCommandSucceeded
            && isOk(hoodServo.setEnabled(true))
            && isOk(hoodServo.setPowered(true));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.positionDeg = positionDeg;
    inputs.pulseWidthUs = pulseWidthUs;
    if (refreshCounter % POWER_STATUS_PERIOD_LOOPS == 0) {
      ServoHub servoHub = getConfiguredHoodServoHub();
      inputs.hubDeviceVoltageVolts = servoHub.getDeviceVoltage();
      inputs.hubDeviceCurrentAmps = servoHub.getDeviceCurrent();
      inputs.servoVoltageVolts = servoHub.getServoVoltage();
      inputs.channelCurrentAmps = hoodServo.getCurrent();
      inputs.hubHasActiveFault = servoHub.hasActiveFault();
      inputs.hubHasActiveWarning = servoHub.hasActiveWarning();
    }
    inputs.connected =
        hoodServoHubConfigured && lastCommandSucceeded && inputs.hubDeviceVoltageVolts > 0.0;
    refreshCounter++;
  }

  @Override
  public void setAngleDeg(double hoodAngleDeg) {
    positionDeg = MathUtil.clamp(hoodAngleDeg, minHoodAngle, maxHoodAngle);
    pulseWidthUs = hoodAngleToPulseWidthUs(positionDeg, invertPulseDirection);
    lastCommandSucceeded = isOk(hoodServo.setPulseWidth(pulseWidthUs));
  }

  private static synchronized ServoHub getConfiguredHoodServoHub() {
    if (hoodServoHub == null) {
      hoodServoHub = new ServoHub(HOOD_SERVO_HUB_CAN_ID);
    }

    if (!hoodServoHubConfigured) {
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
      hoodServoHubConfigured =
          isOk(hoodServoHub.configure(hubConfig, ResetMode.kNoResetSafeParameters));
    }
    return hoodServoHub;
  }

  private static boolean isOk(REVLibError error) {
    return error == REVLibError.kOk;
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
