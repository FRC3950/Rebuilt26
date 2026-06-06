package frc.robot.subsystems.turret.turret_base;

import static frc.robot.Constants.SubsystemConstants.Turret.flywheelGearRatio;
import static frc.robot.Constants.SubsystemConstants.Turret.maxFlywheelRps;
import static frc.robot.Constants.SubsystemConstants.Turret.minFlywheelRps;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.BatteryLogger;
import org.littletonrobotics.junction.Logger;

public class Flywheels {
  private static final int POWER_STATUS_PERIOD_LOOPS = 5;

  private final TalonFX flywheel;
  private final TalonFX flywheelFollower;

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;
  private final StatusSignal<Voltage> leaderSupplyVoltage;
  private final StatusSignal<Current> leaderSupplyCurrent;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerSupplyVoltage;
  private final StatusSignal<Current> followerSupplyCurrent;
  private double targetRps = 0.0;
  private int refreshCounter = 0;

  public Flywheels(
      int flywheelID, TalonFXConfiguration flywheelConfig, int flywheelFollowerID, CANBus canbus) {
    flywheel = new TalonFX(flywheelID, canbus);
    flywheelFollower = new TalonFX(flywheelFollowerID, canbus);
    leaderVelocity = flywheel.getVelocity();
    leaderAppliedVolts = flywheel.getMotorVoltage();
    leaderCurrent = flywheel.getStatorCurrent();
    leaderSupplyVoltage = flywheel.getSupplyVoltage();
    leaderSupplyCurrent = flywheel.getSupplyCurrent();
    followerVelocity = flywheelFollower.getVelocity();
    followerSupplyVoltage = flywheelFollower.getSupplyVoltage();
    followerSupplyCurrent = flywheelFollower.getSupplyCurrent();

    flywheel.getConfigurator().apply(flywheelConfig);
    flywheelFollower.getConfigurator().apply(flywheelConfig);
    flywheelFollower.setControl(new Follower(flywheelID, MotorAlignmentValue.Opposed));
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0,
        leaderSupplyVoltage,
        leaderSupplyCurrent,
        followerSupplyVoltage,
        followerSupplyCurrent);
    ParentDevice.optimizeBusUtilizationForAll(flywheel, flywheelFollower);
  }

  public void periodic(String logKey) {
    boolean refreshPowerSignals = refreshCounter % POWER_STATUS_PERIOD_LOOPS == 0;
    BaseStatusSignal.refreshAll(
        leaderVelocity, leaderAppliedVolts, leaderCurrent, followerVelocity);
    if (refreshPowerSignals) {
      BaseStatusSignal.refreshAll(
          leaderSupplyVoltage, leaderSupplyCurrent, followerSupplyVoltage, followerSupplyCurrent);
    }

    Logger.recordOutput(logKey + "/LeaderVelocityRps", getMeasuredVelocityRps());
    Logger.recordOutput(logKey + "/FollowerVelocityRps", getFollowerVelocityRps());
    Logger.recordOutput(logKey + "/AppliedVolts", leaderAppliedVolts.getValueAsDouble());
    Logger.recordOutput(logKey + "/CurrentAmps", leaderCurrent.getValueAsDouble());
    if (refreshPowerSignals) {
      Logger.recordOutput(
          logKey + "/LeaderSupplyVoltageVolts", leaderSupplyVoltage.getValueAsDouble());
      Logger.recordOutput(
          logKey + "/LeaderSupplyCurrentAmps", leaderSupplyCurrent.getValueAsDouble());
      Logger.recordOutput(
          logKey + "/FollowerSupplyVoltageVolts", followerSupplyVoltage.getValueAsDouble());
      Logger.recordOutput(
          logKey + "/FollowerSupplyCurrentAmps", followerSupplyCurrent.getValueAsDouble());
    }
    BatteryLogger.reportCurrentUsage(
        logKey, leaderSupplyCurrent.getValueAsDouble(), followerSupplyCurrent.getValueAsDouble());
    refreshCounter++;
  }

  public void setTargetRps(double flywheelSpeedRps) {
    targetRps = MathUtil.clamp(flywheelSpeedRps, minFlywheelRps, maxFlywheelRps);
    double targetVel = targetRps * flywheelGearRatio;
    flywheel.setControl(
        velocityVoltage.withVelocity(
            MathUtil.clamp(
                targetVel,
                minFlywheelRps * flywheelGearRatio,
                maxFlywheelRps * flywheelGearRatio)));
  }

  public double getVelocityRps() {
    return leaderVelocity.getValueAsDouble() / flywheelGearRatio;
  }

  public double getMeasuredVelocityRps() {
    return getVelocityRps();
  }

  public double getFollowerVelocityRps() {
    return followerVelocity.getValueAsDouble() / flywheelGearRatio;
  }

  public double getTargetRps() {
    return targetRps;
  }

  public boolean isReadyForFeed(double toleranceRps) {
    return Math.abs(getMeasuredVelocityRps() - targetRps) <= toleranceRps
        && Math.abs(Math.abs(getFollowerVelocityRps()) - targetRps) <= toleranceRps;
  }
}
