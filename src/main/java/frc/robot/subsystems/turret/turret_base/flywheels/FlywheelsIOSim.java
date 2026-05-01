package frc.robot.subsystems.turret.turret_base.flywheels;

import static frc.robot.sim.MechanismSimConstants.LOOP_PERIOD_SECS;
import static frc.robot.sim.MechanismSimConstants.MAX_VOLTAGE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.sim.MechanismSimConstants;

public class FlywheelsIOSim implements FlywheelsIO {
  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              MechanismSimConstants.Turret.FLYWHEEL_GEARBOX,
              MechanismSimConstants.Turret.FLYWHEEL_MOI_KG_METERS_SQUARED,
              frc.robot.Constants.SubsystemConstants.Turret.flywheelGearRatio),
          MechanismSimConstants.Turret.FLYWHEEL_GEARBOX);
  private final PIDController controller =
      new PIDController(
          MechanismSimConstants.Turret.FLYWHEEL_KP, 0.0, MechanismSimConstants.Turret.FLYWHEEL_KD);

  private double targetRps = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    double velocityRps = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
    double requestedVolts =
        controller.calculate(velocityRps, targetRps)
            + MechanismSimConstants.Turret.FLYWHEEL_KV_VOLTS_PER_RPS * targetRps;
    double maxVoltageStep =
        MAX_VOLTAGE
            * LOOP_PERIOD_SECS
            / MechanismSimConstants.Turret.FLYWHEEL_VOLTAGE_RAMP_PERIOD_SECS;
    appliedVolts += MathUtil.clamp(requestedVolts - appliedVolts, -maxVoltageStep, maxVoltageStep);
    appliedVolts = MathUtil.clamp(appliedVolts, -MAX_VOLTAGE, MAX_VOLTAGE);

    sim.setInputVoltage(appliedVolts);
    sim.update(LOOP_PERIOD_SECS);

    inputs.leaderConnected = true;
    inputs.followerConnected = true;
    inputs.leaderVelocityRps = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
    inputs.followerVelocityRps = -inputs.leaderVelocityRps;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void setTargetRps(double flywheelSpeedRps) {
    targetRps = flywheelSpeedRps;
  }
}
