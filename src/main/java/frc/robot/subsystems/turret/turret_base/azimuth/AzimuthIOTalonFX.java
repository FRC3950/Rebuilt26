package frc.robot.subsystems.turret.turret_base.azimuth;

import static frc.robot.Constants.SubsystemConstants.Turret.TURRET_CANDI_ID;
import static frc.robot.Constants.SubsystemConstants.Turret.azimuthGearRatio;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class AzimuthIOTalonFX implements AzimuthIO {
  private static final int POWER_STATUS_PERIOD_LOOPS = 5;

  private final TalonFX azimuth;
  private final CANdi zeroingCandi;
  private final boolean usesCandiS1;
  private final PositionVoltage azimuthControl = new PositionVoltage(0.0).withUpdateFreqHz(250);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Voltage> supplyVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private int refreshCounter = 0;

  public AzimuthIOTalonFX(
      int azimuthID, TalonFXConfiguration azimuthConfig, CANBus canbus, boolean usesCandiS1) {
    this(azimuthID, azimuthConfig, canbus, usesCandiS1, true);
  }

  public AzimuthIOTalonFX(
      int azimuthID,
      TalonFXConfiguration azimuthConfig,
      CANBus canbus,
      boolean usesCandiS1,
      boolean enableZeroSwitch) {
    this.usesCandiS1 = usesCandiS1;
    azimuth = new TalonFX(azimuthID, canbus);
    zeroingCandi = enableZeroSwitch ? new CANdi(TURRET_CANDI_ID, canbus) : null;
    position = azimuth.getPosition();
    velocity = azimuth.getVelocity();
    appliedVolts = azimuth.getMotorVoltage();
    current = azimuth.getStatorCurrent();
    supplyVoltage = azimuth.getSupplyVoltage();
    supplyCurrent = azimuth.getSupplyCurrent();
    azimuth.getConfigurator().apply(azimuthConfig);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, supplyVoltage, supplyCurrent);
    ParentDevice.optimizeBusUtilizationForAll(azimuth);
  }

  @Override
  public void updateInputs(AzimuthIOInputs inputs) {
    boolean refreshPowerSignals = refreshCounter % POWER_STATUS_PERIOD_LOOPS == 0;
    var status = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);
    if (refreshPowerSignals) {
      BaseStatusSignal.refreshAll(supplyVoltage, supplyCurrent);
    }
    inputs.connected = connectedDebounce.calculate(status.isOK());
    inputs.positionDeg = Units.rotationsToDegrees(position.getValueAsDouble() / azimuthGearRatio);
    inputs.velocityDegPerSec =
        Units.rotationsToDegrees(velocity.getValueAsDouble() / azimuthGearRatio);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    if (refreshPowerSignals) {
      inputs.supplyVoltageVolts = supplyVoltage.getValueAsDouble();
      inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    }
    if (zeroingCandi == null) {
      inputs.zeroSwitchClosed = false;
      refreshCounter++;
      return;
    }
    inputs.zeroSwitchClosed =
        usesCandiS1
            ? Boolean.TRUE.equals(zeroingCandi.getS1Closed().getValue())
            : Boolean.TRUE.equals(zeroingCandi.getS2Closed().getValue());
    refreshCounter++;
  }

  @Override
  public void setTargetAngleDeg(double targetAngleDeg) {
    setTargetAngleDeg(targetAngleDeg, 0.0);
  }

  @Override
  public void setTargetAngleDeg(double targetAngleDeg, double targetVelocityDegPerSec) {
    double motorRotations = Units.degreesToRotations(targetAngleDeg) * azimuthGearRatio;
    double motorRotationsPerSecond =
        Units.degreesToRotations(targetVelocityDegPerSec) * azimuthGearRatio;
    azimuth.setControl(
        azimuthControl.withPosition(motorRotations).withVelocity(motorRotationsPerSecond));
  }

  @Override
  public void zeroPosition() {
    azimuth.setPosition(0.0);
  }
}
