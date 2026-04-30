package frc.robot.subsystems.intake;

import static frc.robot.sim.MechanismSimConstants.LOOP_PERIOD_SECS;
import static frc.robot.sim.MechanismSimConstants.MAX_VOLTAGE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.sim.MechanismSimConstants;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim rollerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              MechanismSimConstants.Intake.ROLLER_GEARBOX,
              MechanismSimConstants.Intake.ROLLER_MOI_KG_METERS_SQUARED,
              MechanismSimConstants.Intake.ROLLER_GEAR_RATIO),
          MechanismSimConstants.Intake.ROLLER_GEARBOX);
  private final SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          MechanismSimConstants.Intake.PIVOT_GEARBOX,
          MechanismSimConstants.Intake.PIVOT_GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(
              MechanismSimConstants.Intake.PIVOT_LENGTH_METERS,
              MechanismSimConstants.Intake.PIVOT_MASS_KG),
          MechanismSimConstants.Intake.PIVOT_LENGTH_METERS,
          MechanismSimConstants.Intake.PIVOT_MIN_ANGLE_RAD,
          MechanismSimConstants.Intake.PIVOT_MAX_ANGLE_RAD,
          true,
          MechanismSimConstants.Intake.PIVOT_STARTING_ANGLE_RAD);

  private final PIDController rollerController =
      new PIDController(
          MechanismSimConstants.Intake.ROLLER_KP, 0.0, MechanismSimConstants.Intake.ROLLER_KD);
  private final PIDController pivotController =
      new PIDController(MechanismSimConstants.Intake.PIVOT_KP, 0.0, MechanismSimConstants.Intake.PIVOT_KD);

  private double rollerSetpointRps = 0.0;
  private double pivotSetpoint = frc.robot.Constants.SubsystemConstants.Intake.upPos;
  private double pivotSensorOffset = 0.0;
  private double rollerAppliedVolts = 0.0;
  private double pivotAppliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    double rollerVelocityRps = Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec());
    rollerAppliedVolts =
        MathUtil.clamp(
            rollerController.calculate(rollerVelocityRps, rollerSetpointRps)
                + MechanismSimConstants.Intake.ROLLER_KV_VOLTS_PER_RPS * rollerSetpointRps,
            -MAX_VOLTAGE,
            MAX_VOLTAGE);

    double pivotSetpointRad = MechanismSimConstants.Intake.pivotPositionToAngleRad(pivotSetpoint);
    pivotAppliedVolts =
        MathUtil.clamp(
            pivotController.calculate(pivotSim.getAngleRads(), pivotSetpointRad)
                + MechanismSimConstants.Intake.PIVOT_KG_VOLTS * Math.cos(pivotSim.getAngleRads()),
            -MAX_VOLTAGE,
            MAX_VOLTAGE);

    rollerSim.setInputVoltage(rollerAppliedVolts);
    pivotSim.setInputVoltage(pivotAppliedVolts);
    rollerSim.update(LOOP_PERIOD_SECS);
    pivotSim.update(LOOP_PERIOD_SECS);

    inputs.rollerConnected = true;
    inputs.rollerVelocityRps = Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec());
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());

    inputs.pivotConnected = true;
    inputs.pivotPosition =
        MechanismSimConstants.Intake.pivotAngleRadToPosition(pivotSim.getAngleRads())
            - pivotSensorOffset;
    inputs.pivotVelocityRps = Units.radiansToRotations(pivotSim.getVelocityRadPerSec());
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotCurrentAmps = Math.abs(pivotSim.getCurrentDrawAmps());
  }

  @Override
  public void setRollerVelocity(double velocityRps) {
    rollerSetpointRps = velocityRps;
  }

  @Override
  public void setPivotPosition(double position) {
    pivotSetpoint = position;
  }

  @Override
  public void zeroPivotPosition() {
    pivotSensorOffset = MechanismSimConstants.Intake.pivotAngleRadToPosition(pivotSim.getAngleRads());
    pivotSetpoint = frc.robot.Constants.SubsystemConstants.Intake.upPos;
  }
}
