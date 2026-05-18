package frc.robot.subsystems.turret.turret_base.azimuth;

import static frc.robot.Constants.SubsystemConstants.Turret.TURRET_CANDI_ID;
import static frc.robot.Constants.SubsystemConstants.Turret.azimuthGearRatio;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class AzimuthIOTalonFX implements AzimuthIO {
  private final TalonFX azimuth;
  private final CANdi zeroingCandi;
  private final boolean usesCandiS1;
  private final MotionMagicVoltage azimuthControl = new MotionMagicVoltage(0.0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

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
    azimuth.getConfigurator().apply(azimuthConfig);
  }

  @Override
  public void updateInputs(AzimuthIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);
    inputs.connected = connectedDebounce.calculate(status.isOK());
    inputs.positionDeg = Units.rotationsToDegrees(position.getValueAsDouble() / azimuthGearRatio);
    inputs.velocityDegPerSec =
        Units.rotationsToDegrees(velocity.getValueAsDouble() / azimuthGearRatio);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    if (zeroingCandi == null) {
      inputs.zeroSwitchClosed = false;
      return;
    }
    inputs.zeroSwitchClosed =
        usesCandiS1
            ? Boolean.TRUE.equals(zeroingCandi.getS1Closed().getValue())
            : Boolean.TRUE.equals(zeroingCandi.getS2Closed().getValue());
  }

  @Override
  public void setTargetAngleDeg(double targetAngleDeg) {
    double motorRotations = Units.degreesToRotations(targetAngleDeg) * azimuthGearRatio;
    azimuth.setControl(azimuthControl.withPosition(motorRotations));
  }

  @Override
  public void zeroPosition() {
    azimuth.setPosition(0.0);
  }
}
