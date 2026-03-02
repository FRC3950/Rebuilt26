package frc.robot.subsystems.turret.turret_base;

import static frc.robot.Constants.SubsystemConstants.Turret.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;

public class Azimuth {
  public enum LimitSwitchChannel {
    S1,
    S2
  }

  private static CANdi turretCANdi;

  private final TalonFX azimuth;
  private final MotionMagicVoltage azimuthControl = new MotionMagicVoltage(0.0);
  private final StatusSignal<Boolean> limitSwitchClosedSignal;
  private boolean wasLimitSwitchClosed = false;

  private double lastSetpointDeg = 0.0;

  public Azimuth(
      int azimuthID,
      TalonFXConfiguration azimuthConfig,
      CANBus canbus,
      LimitSwitchChannel limitSwitchChannel) {
    azimuth = new TalonFX(azimuthID, canbus);
    azimuth.getConfigurator().apply(azimuthConfig);

    CANdi candi = getConfiguredTurretCANdi(canbus);
    limitSwitchClosedSignal =
        switch (limitSwitchChannel) {
          case S1 -> candi.getS1Closed();
          case S2 -> candi.getS2Closed();
        };
  }

  public void setTargetAngleDeg(double targetAngleDeg) {
    lastSetpointDeg = targetAngleDeg;
    double motorRotations = Units.degreesToRotations(targetAngleDeg) * azimuthGearRatio;
    azimuth.setControl(azimuthControl.withPosition(motorRotations));
  }

  public double getMotorAngleDeg() {
    double azimuthRotations = azimuth.getPosition().getValueAsDouble();
    return Units.rotationsToDegrees(azimuthRotations / azimuthGearRatio);
  }

  public double getMotorVelocityDegPerSec() {
    return Units.rotationsToDegrees(azimuth.getVelocity().getValueAsDouble() / azimuthGearRatio);
  }

  public double getSetpointDeg() {
    return lastSetpointDeg;
  }

  public void syncMotorPositionToLimitSwitchIfTriggered() {
    boolean limitSwitchClosed =
        BaseStatusSignal.refreshAll(limitSwitchClosedSignal).isOK()
            && Boolean.TRUE.equals(limitSwitchClosedSignal.getValue());
    if (limitSwitchClosed && !wasLimitSwitchClosed) {
      setMotorPositionDeg(TURRET_LIMIT_SWITCH_ANGLE_DEG);
    }
    wasLimitSwitchClosed = limitSwitchClosed;
  }

  public void zeroMotorPosition() {
    azimuth.setPosition(0.0);
    lastSetpointDeg = 0.0;
  }

  private void setMotorPositionDeg(double motorAngleDeg) {
    azimuth.setPosition(Units.degreesToRotations(motorAngleDeg) * azimuthGearRatio);
    lastSetpointDeg = motorAngleDeg;
  }

  private static synchronized CANdi getConfiguredTurretCANdi(CANBus canbus) {
    if (turretCANdi == null) {
      turretCANdi = new CANdi(TURRET_CANDI_ID, canbus);
    }
    return turretCANdi;
  }
}
