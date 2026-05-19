package frc.robot.subsystems.intake;

import static frc.robot.Constants.SubsystemConstants.CANivore;
import static frc.robot.Constants.SubsystemConstants.Intake.intakeConfig;
import static frc.robot.Constants.SubsystemConstants.Intake.intakeMotorID;
import static frc.robot.Constants.SubsystemConstants.Intake.pivotConfig;
import static frc.robot.Constants.SubsystemConstants.Intake.pivotMotorID;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  private static final int POWER_STATUS_PERIOD_LOOPS = 5;

  private final TalonFX intakeMotor = new TalonFX(intakeMotorID, CANivore);
  private final TalonFX pivotMotor = new TalonFX(pivotMotorID, CANivore);
  private final MotionMagicVoltage pivotControl = new MotionMagicVoltage(0);
  private final VelocityVoltage intakeControl = new VelocityVoltage(0);

  private final StatusSignal<AngularVelocity> rollerVelocity = intakeMotor.getVelocity();
  private final StatusSignal<Voltage> rollerAppliedVolts = intakeMotor.getMotorVoltage();
  private final StatusSignal<Current> rollerCurrent = intakeMotor.getStatorCurrent();
  private final StatusSignal<Voltage> rollerSupplyVoltage = intakeMotor.getSupplyVoltage();
  private final StatusSignal<Current> rollerSupplyCurrent = intakeMotor.getSupplyCurrent();
  private final StatusSignal<Angle> pivotPosition = pivotMotor.getPosition();
  private final StatusSignal<AngularVelocity> pivotVelocity = pivotMotor.getVelocity();
  private final StatusSignal<Voltage> pivotAppliedVolts = pivotMotor.getMotorVoltage();
  private final StatusSignal<Current> pivotCurrent = pivotMotor.getStatorCurrent();
  private final StatusSignal<Voltage> pivotSupplyVoltage = pivotMotor.getSupplyVoltage();
  private final StatusSignal<Current> pivotSupplyCurrent = pivotMotor.getSupplyCurrent();

  private final Debouncer rollerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer pivotConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private int refreshCounter = 0;

  public IntakeIOTalonFX() {
    pivotMotor.getConfigurator().apply(pivotConfig);
    intakeMotor.getConfigurator().apply(intakeConfig);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        rollerVelocity,
        rollerAppliedVolts,
        rollerCurrent,
        pivotPosition,
        pivotVelocity,
        pivotAppliedVolts,
        pivotCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0, rollerSupplyVoltage, rollerSupplyCurrent, pivotSupplyVoltage, pivotSupplyCurrent);
    ParentDevice.optimizeBusUtilizationForAll(intakeMotor, pivotMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    boolean refreshPowerSignals = refreshCounter % POWER_STATUS_PERIOD_LOOPS == 0;
    var rollerStatus =
        BaseStatusSignal.refreshAll(rollerVelocity, rollerAppliedVolts, rollerCurrent);
    var pivotStatus =
        BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, pivotAppliedVolts, pivotCurrent);
    if (refreshPowerSignals) {
      BaseStatusSignal.refreshAll(
          rollerSupplyVoltage, rollerSupplyCurrent, pivotSupplyVoltage, pivotSupplyCurrent);
    }

    inputs.rollerConnected = rollerConnectedDebounce.calculate(rollerStatus.isOK());
    inputs.rollerVelocityRps = rollerVelocity.getValueAsDouble();
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
    if (refreshPowerSignals) {
      inputs.rollerSupplyVoltageVolts = rollerSupplyVoltage.getValueAsDouble();
      inputs.rollerSupplyCurrentAmps = rollerSupplyCurrent.getValueAsDouble();
    }

    inputs.pivotConnected = pivotConnectedDebounce.calculate(pivotStatus.isOK());
    inputs.pivotPosition = pivotPosition.getValueAsDouble();
    inputs.pivotVelocityRps = pivotVelocity.getValueAsDouble();
    inputs.pivotAppliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.pivotCurrentAmps = pivotCurrent.getValueAsDouble();
    if (refreshPowerSignals) {
      inputs.pivotSupplyVoltageVolts = pivotSupplyVoltage.getValueAsDouble();
      inputs.pivotSupplyCurrentAmps = pivotSupplyCurrent.getValueAsDouble();
    }
    refreshCounter++;
  }

  @Override
  public void setRollerVelocity(double velocityRps) {
    intakeMotor.setControl(intakeControl.withVelocity(velocityRps));
  }

  @Override
  public void setPivotPosition(double position) {
    pivotMotor.setControl(pivotControl.withPosition(position));
  }

  @Override
  public void zeroPivotPosition() {
    pivotMotor.setPosition(0.0);
  }
}
