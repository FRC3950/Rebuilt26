// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.Constants.SubsystemConstants.CANivore;
import static frc.robot.Constants.SubsystemConstants.Intake.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BatteryLogger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final int POWER_STATUS_PERIOD_LOOPS = 5;

  private final TalonFX intakeMotor;
  private final TalonFX pivotMotor;
  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);
  private final VelocityVoltage intakeControlRequest = new VelocityVoltage(0);
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerCurrent;
  private final StatusSignal<Voltage> rollerSupplyVoltage;
  private final StatusSignal<Current> rollerSupplyCurrent;
  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotCurrent;
  private final StatusSignal<Voltage> pivotSupplyVoltage;
  private final StatusSignal<Current> pivotSupplyCurrent;
  private double commandedRollerSpeed = 0.0;
  private double pivotSetpoint = upPos;
  private boolean isIntaking = false;
  private int refreshCounter = 0;

  /** Creates a new intake. */
  public Intake() {
    intakeMotor = new TalonFX(intakeMotorID, CANivore);
    pivotMotor = new TalonFX(pivotMotorID, CANivore);
    rollerVelocity = intakeMotor.getVelocity();
    rollerAppliedVolts = intakeMotor.getMotorVoltage();
    rollerCurrent = intakeMotor.getStatorCurrent();
    rollerSupplyVoltage = intakeMotor.getSupplyVoltage();
    rollerSupplyCurrent = intakeMotor.getSupplyCurrent();
    pivotPosition = pivotMotor.getPosition();
    pivotVelocity = pivotMotor.getVelocity();
    pivotAppliedVolts = pivotMotor.getMotorVoltage();
    pivotCurrent = pivotMotor.getStatorCurrent();
    pivotSupplyVoltage = pivotMotor.getSupplyVoltage();
    pivotSupplyCurrent = pivotMotor.getSupplyCurrent();

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
  public void periodic() {
    boolean refreshPowerSignals = refreshCounter % POWER_STATUS_PERIOD_LOOPS == 0;
    BaseStatusSignal.refreshAll(rollerVelocity, rollerAppliedVolts, rollerCurrent);
    BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, pivotAppliedVolts, pivotCurrent);
    if (refreshPowerSignals) {
      BaseStatusSignal.refreshAll(
          rollerSupplyVoltage, rollerSupplyCurrent, pivotSupplyVoltage, pivotSupplyCurrent);
    }

    Logger.recordOutput("Intake/Roller Applied Volts", rollerAppliedVolts.getValueAsDouble());
    Logger.recordOutput("Intake/Roller Current Amps", rollerCurrent.getValueAsDouble());
    Logger.recordOutput("Intake/Pivot Velocity Rps", pivotVelocity.getValueAsDouble());
    Logger.recordOutput("Intake/Pivot Applied Volts", pivotAppliedVolts.getValueAsDouble());
    Logger.recordOutput("Intake/Pivot Current Amps", pivotCurrent.getValueAsDouble());
    if (refreshPowerSignals) {
      Logger.recordOutput("Intake/Roller Supply Voltage", rollerSupplyVoltage.getValueAsDouble());
      Logger.recordOutput("Intake/Roller Supply Current", rollerSupplyCurrent.getValueAsDouble());
      Logger.recordOutput("Intake/Pivot Supply Voltage", pivotSupplyVoltage.getValueAsDouble());
      Logger.recordOutput("Intake/Pivot Supply Current", pivotSupplyCurrent.getValueAsDouble());
    }
    BatteryLogger.reportCurrentUsage("Intake/Roller", rollerSupplyCurrent.getValueAsDouble());
    BatteryLogger.reportCurrentUsage("Intake/Pivot", pivotSupplyCurrent.getValueAsDouble());
    refreshCounter++;
  }

  public void setIntakeSpeed(double speed) {
    commandedRollerSpeed = speed;
    isIntaking = speed > 0.0;
    intakeMotor.setControl(intakeControlRequest.withVelocity(speed));
  }

  public void reverseIntake() {
    intakeMotor.setControl(intakeControlRequest.withVelocity(unjamSpeed));
  }

  public void startIntake() {
    isIntaking = true;
    setIntakeSpeed(mintakeSpeed + 8);
  }

  public void stopIntake() {
    isIntaking = false;
    setIntakeSpeed(0);
  }

  public boolean isIntaking() {
    return isIntaking;
  }

  @AutoLogOutput(key = "Intake/Current Speed")
  public double getIntakeCurrentSpeed() {
    return rollerVelocity.getValueAsDouble();
  }

  public double getMeasuredRollerSpeed() {
    return rollerVelocity.getValueAsDouble();
  }

  public void setPivotPosition(double position) {
    pivotSetpoint = position;
    pivotMotor.setControl(mmRequest.withPosition(position));
  }

  public void extend() {
    setPivotPosition(downPos);
  }

  public void retract() {
    setPivotPosition(upPos);
  }

  public boolean isAcceptablePosition(double targetPos) {
    return Math.abs(pivotPosition.getValueAsDouble() - targetPos) < 0.05;
  }

  @AutoLogOutput(key = "Intake/Pivot Position")
  public double getPivotPosition() {
    return pivotPosition.getValueAsDouble();
  }

  public double getCommandedRollerSpeed() {
    return commandedRollerSpeed;
  }

  @AutoLogOutput(key = "Intake/Pivot Setpoint")
  public double getPivotSetpoint() {
    return pivotSetpoint;
  }

  @AutoLogOutput(key = "Intake/Commanded Roller Speed")
  public double getLoggedCommandedRollerSpeed() {
    return getCommandedRollerSpeed();
  }

  public boolean isPivotCommandedDown() {
    return Math.abs(pivotSetpoint - downPos) < 1e-9;
  }

  public boolean isPivotMeasuredDown() {
    return isAcceptablePosition(downPos);
  }

  public Command extendCommand() {
    return this.runOnce(this::extend);
  }

  public Command retractCommand() {
    return this.runOnce(this::retract);
  }

  public Command onIntake() {
    return this.runOnce(this::startIntake);
  }

  public Command zeroIntake() {
    return Commands.runOnce(() -> pivotMotor.setPosition(0), this);
  }

  public Command offIntake() {
    return this.runOnce(this::stopIntake);
  }
}
