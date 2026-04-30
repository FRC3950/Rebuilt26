package frc.robot.subsystems.turret.turret_base.flywheels;

import static frc.robot.Constants.SubsystemConstants.Turret.flywheelGearRatio;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelsIOTalonFX implements FlywheelsIO {
  private final TalonFX flywheel;
  private final TalonFX flywheelFollower;
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final Debouncer leaderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public FlywheelsIOTalonFX(
      int flywheelID, TalonFXConfiguration flywheelConfig, int flywheelFollowerID, CANBus canbus) {
    flywheel = new TalonFX(flywheelID, canbus);
    flywheelFollower = new TalonFX(flywheelFollowerID, canbus);
    leaderVelocity = flywheel.getVelocity();
    leaderAppliedVolts = flywheel.getMotorVoltage();
    leaderCurrent = flywheel.getStatorCurrent();
    followerVelocity = flywheelFollower.getVelocity();

    flywheel.getConfigurator().apply(flywheelConfig);
    flywheelFollower.setControl(new Follower(flywheelID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    var leaderStatus = BaseStatusSignal.refreshAll(leaderVelocity, leaderAppliedVolts, leaderCurrent);
    var followerStatus = BaseStatusSignal.refreshAll(followerVelocity);

    inputs.leaderConnected = leaderConnectedDebounce.calculate(leaderStatus.isOK());
    inputs.followerConnected = followerConnectedDebounce.calculate(followerStatus.isOK());
    inputs.leaderVelocityRps = leaderVelocity.getValueAsDouble() / flywheelGearRatio;
    inputs.followerVelocityRps = followerVelocity.getValueAsDouble() / flywheelGearRatio;
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps = leaderCurrent.getValueAsDouble();
  }

  @Override
  public void setTargetRps(double flywheelSpeedRps) {
    flywheel.setControl(velocityVoltage.withVelocity(flywheelSpeedRps * flywheelGearRatio));
  }
}
