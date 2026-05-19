package frc.robot.subsystems.indexer;

import static frc.robot.Constants.SubsystemConstants.CANivore;
import static frc.robot.Constants.SubsystemConstants.Indexer.hotdogConfig;
import static frc.robot.Constants.SubsystemConstants.Indexer.hotdogMotorID;
import static frc.robot.Constants.SubsystemConstants.Indexer.indexerConfig;
import static frc.robot.Constants.SubsystemConstants.Indexer.indexerMotorID;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOTalonFX implements IndexerIO {
  private static final int POWER_STATUS_PERIOD_LOOPS = 5;

  private final TalonFX hotdogMotor = new TalonFX(hotdogMotorID, CANivore);
  private final TalonFX indexerMotor = new TalonFX(indexerMotorID, CANivore);

  private final VelocityVoltage indexerControl = new VelocityVoltage(0);
  private final VelocityVoltage hotdogControl = new VelocityVoltage(0);

  private final StatusSignal<AngularVelocity> indexerVelocity = indexerMotor.getVelocity();
  private final StatusSignal<Voltage> indexerAppliedVolts = indexerMotor.getMotorVoltage();
  private final StatusSignal<Current> indexerCurrent = indexerMotor.getStatorCurrent();
  private final StatusSignal<Voltage> indexerSupplyVoltage = indexerMotor.getSupplyVoltage();
  private final StatusSignal<Current> indexerSupplyCurrent = indexerMotor.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> hotdogVelocity = hotdogMotor.getVelocity();
  private final StatusSignal<Voltage> hotdogAppliedVolts = hotdogMotor.getMotorVoltage();
  private final StatusSignal<Current> hotdogCurrent = hotdogMotor.getStatorCurrent();
  private final StatusSignal<Voltage> hotdogSupplyVoltage = hotdogMotor.getSupplyVoltage();
  private final StatusSignal<Current> hotdogSupplyCurrent = hotdogMotor.getSupplyCurrent();

  private final Debouncer indexerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer hotdogConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private int refreshCounter = 0;

  public IndexerIOTalonFX() {
    hotdogMotor.getConfigurator().apply(hotdogConfig);
    indexerMotor.getConfigurator().apply(indexerConfig);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        indexerVelocity,
        indexerAppliedVolts,
        indexerCurrent,
        hotdogVelocity,
        hotdogAppliedVolts,
        hotdogCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0, indexerSupplyVoltage, indexerSupplyCurrent, hotdogSupplyVoltage, hotdogSupplyCurrent);
    ParentDevice.optimizeBusUtilizationForAll(hotdogMotor, indexerMotor);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    boolean refreshPowerSignals = refreshCounter % POWER_STATUS_PERIOD_LOOPS == 0;
    var indexerStatus =
        BaseStatusSignal.refreshAll(indexerVelocity, indexerAppliedVolts, indexerCurrent);
    var hotdogStatus =
        BaseStatusSignal.refreshAll(hotdogVelocity, hotdogAppliedVolts, hotdogCurrent);
    if (refreshPowerSignals) {
      BaseStatusSignal.refreshAll(
          indexerSupplyVoltage, indexerSupplyCurrent, hotdogSupplyVoltage, hotdogSupplyCurrent);
    }

    inputs.indexerConnected = indexerConnectedDebounce.calculate(indexerStatus.isOK());
    inputs.indexerVelocityRps = indexerVelocity.getValueAsDouble();
    inputs.indexerAppliedVolts = indexerAppliedVolts.getValueAsDouble();
    inputs.indexerCurrentAmps = indexerCurrent.getValueAsDouble();
    if (refreshPowerSignals) {
      inputs.indexerSupplyVoltageVolts = indexerSupplyVoltage.getValueAsDouble();
      inputs.indexerSupplyCurrentAmps = indexerSupplyCurrent.getValueAsDouble();
    }

    inputs.hotdogConnected = hotdogConnectedDebounce.calculate(hotdogStatus.isOK());
    inputs.hotdogVelocityRps = hotdogVelocity.getValueAsDouble();
    inputs.hotdogAppliedVolts = hotdogAppliedVolts.getValueAsDouble();
    inputs.hotdogCurrentAmps = hotdogCurrent.getValueAsDouble();
    if (refreshPowerSignals) {
      inputs.hotdogSupplyVoltageVolts = hotdogSupplyVoltage.getValueAsDouble();
      inputs.hotdogSupplyCurrentAmps = hotdogSupplyCurrent.getValueAsDouble();
    }
    refreshCounter++;
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
