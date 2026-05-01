package frc.robot.subsystems.indexer;

import static frc.robot.Constants.SubsystemConstants.CANivore;
import static frc.robot.Constants.SubsystemConstants.Indexer.hotdogConfig;
import static frc.robot.Constants.SubsystemConstants.Indexer.hotdogMotorID;
import static frc.robot.Constants.SubsystemConstants.Indexer.indexerConfig;
import static frc.robot.Constants.SubsystemConstants.Indexer.indexerMotorID;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX hotdogMotor = new TalonFX(hotdogMotorID, CANivore);
  private final TalonFX indexerMotor = new TalonFX(indexerMotorID, CANivore);

  private final VelocityVoltage indexerControl = new VelocityVoltage(0);
  private final VelocityVoltage hotdogControl = new VelocityVoltage(0);

  private final StatusSignal<AngularVelocity> indexerVelocity = indexerMotor.getVelocity();
  private final StatusSignal<Voltage> indexerAppliedVolts = indexerMotor.getMotorVoltage();
  private final StatusSignal<Current> indexerCurrent = indexerMotor.getStatorCurrent();
  private final StatusSignal<AngularVelocity> hotdogVelocity = hotdogMotor.getVelocity();
  private final StatusSignal<Voltage> hotdogAppliedVolts = hotdogMotor.getMotorVoltage();
  private final StatusSignal<Current> hotdogCurrent = hotdogMotor.getStatorCurrent();

  private final Debouncer indexerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer hotdogConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IndexerIOTalonFX() {
    hotdogMotor.getConfigurator().apply(hotdogConfig);
    indexerMotor.getConfigurator().apply(indexerConfig);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    var indexerStatus =
        BaseStatusSignal.refreshAll(indexerVelocity, indexerAppliedVolts, indexerCurrent);
    var hotdogStatus =
        BaseStatusSignal.refreshAll(hotdogVelocity, hotdogAppliedVolts, hotdogCurrent);

    inputs.indexerConnected = indexerConnectedDebounce.calculate(indexerStatus.isOK());
    inputs.indexerVelocityRps = indexerVelocity.getValueAsDouble();
    inputs.indexerAppliedVolts = indexerAppliedVolts.getValueAsDouble();
    inputs.indexerCurrentAmps = indexerCurrent.getValueAsDouble();

    inputs.hotdogConnected = hotdogConnectedDebounce.calculate(hotdogStatus.isOK());
    inputs.hotdogVelocityRps = hotdogVelocity.getValueAsDouble();
    inputs.hotdogAppliedVolts = hotdogAppliedVolts.getValueAsDouble();
    inputs.hotdogCurrentAmps = hotdogCurrent.getValueAsDouble();
  }

  @Override
  public void setIndexerVelocity(double velocityRps) {
    indexerMotor.setControl(indexerControl.withVelocity(velocityRps));
  }

  @Override
  public void setHotdogVelocity(double velocityRps) {
    hotdogMotor.setControl(hotdogControl.withVelocity(velocityRps));
  }
}
