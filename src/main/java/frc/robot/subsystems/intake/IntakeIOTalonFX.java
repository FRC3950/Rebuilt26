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
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intakeMotor = new TalonFX(intakeMotorID, CANivore);
  private final TalonFX pivotMotor = new TalonFX(pivotMotorID, CANivore);
  private final MotionMagicVoltage pivotControl = new MotionMagicVoltage(0);
  private final VelocityVoltage intakeControl = new VelocityVoltage(0);

  private final StatusSignal<AngularVelocity> rollerVelocity = intakeMotor.getVelocity();
  private final StatusSignal<Voltage> rollerAppliedVolts = intakeMotor.getMotorVoltage();
  private final StatusSignal<Current> rollerCurrent = intakeMotor.getStatorCurrent();
  private final StatusSignal<Angle> pivotPosition = pivotMotor.getPosition();
  private final StatusSignal<AngularVelocity> pivotVelocity = pivotMotor.getVelocity();
  private final StatusSignal<Voltage> pivotAppliedVolts = pivotMotor.getMotorVoltage();
  private final StatusSignal<Current> pivotCurrent = pivotMotor.getStatorCurrent();

  private final Debouncer rollerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer pivotConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IntakeIOTalonFX() {
    pivotMotor.getConfigurator().apply(pivotConfig);
    intakeMotor.getConfigurator().apply(intakeConfig);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var rollerStatus = BaseStatusSignal.refreshAll(rollerVelocity, rollerAppliedVolts, rollerCurrent);
    var pivotStatus = BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, pivotAppliedVolts, pivotCurrent);

    inputs.rollerConnected = rollerConnectedDebounce.calculate(rollerStatus.isOK());
    inputs.rollerVelocityRps = rollerVelocity.getValueAsDouble();
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();

    inputs.pivotConnected = pivotConnectedDebounce.calculate(pivotStatus.isOK());
    inputs.pivotPosition = pivotPosition.getValueAsDouble();
    inputs.pivotVelocityRps = pivotVelocity.getValueAsDouble();
    inputs.pivotAppliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.pivotCurrentAmps = pivotCurrent.getValueAsDouble();
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
