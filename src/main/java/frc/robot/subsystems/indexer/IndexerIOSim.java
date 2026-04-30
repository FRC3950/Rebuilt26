package frc.robot.subsystems.indexer;

import static frc.robot.sim.MechanismSimConstants.LOOP_PERIOD_SECS;
import static frc.robot.sim.MechanismSimConstants.MAX_VOLTAGE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.sim.MechanismSimConstants;

public class IndexerIOSim implements IndexerIO {
  private final DCMotorSim indexerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              MechanismSimConstants.Indexer.INDEXER_GEARBOX,
              MechanismSimConstants.Indexer.INDEXER_MOI_KG_METERS_SQUARED,
              MechanismSimConstants.Indexer.INDEXER_GEAR_RATIO),
          MechanismSimConstants.Indexer.INDEXER_GEARBOX);
  private final DCMotorSim hotdogSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              MechanismSimConstants.Indexer.HOTDOG_GEARBOX,
              MechanismSimConstants.Indexer.HOTDOG_MOI_KG_METERS_SQUARED,
              MechanismSimConstants.Indexer.HOTDOG_GEAR_RATIO),
          MechanismSimConstants.Indexer.HOTDOG_GEARBOX);

  private final PIDController indexerController =
      new PIDController(
          MechanismSimConstants.Indexer.INDEXER_KP, 0.0, MechanismSimConstants.Indexer.INDEXER_KD);
  private final PIDController hotdogController =
      new PIDController(
          MechanismSimConstants.Indexer.HOTDOG_KP, 0.0, MechanismSimConstants.Indexer.HOTDOG_KD);

  private double indexerSetpointRps = 0.0;
  private double hotdogSetpointRps = 0.0;
  private double indexerAppliedVolts = 0.0;
  private double hotdogAppliedVolts = 0.0;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    double indexerVelocityRps = Units.radiansToRotations(indexerSim.getAngularVelocityRadPerSec());
    double hotdogVelocityRps = Units.radiansToRotations(hotdogSim.getAngularVelocityRadPerSec());

    indexerAppliedVolts =
        MathUtil.clamp(
            indexerController.calculate(indexerVelocityRps, indexerSetpointRps)
                + MechanismSimConstants.Indexer.INDEXER_KV_VOLTS_PER_RPS * indexerSetpointRps,
            -MAX_VOLTAGE,
            MAX_VOLTAGE);
    hotdogAppliedVolts =
        MathUtil.clamp(
            hotdogController.calculate(hotdogVelocityRps, hotdogSetpointRps)
                + MechanismSimConstants.Indexer.HOTDOG_KV_VOLTS_PER_RPS * hotdogSetpointRps,
            -MAX_VOLTAGE,
            MAX_VOLTAGE);

    indexerSim.setInputVoltage(indexerAppliedVolts);
    hotdogSim.setInputVoltage(hotdogAppliedVolts);
    indexerSim.update(LOOP_PERIOD_SECS);
    hotdogSim.update(LOOP_PERIOD_SECS);

    inputs.indexerConnected = true;
    inputs.indexerVelocityRps = Units.radiansToRotations(indexerSim.getAngularVelocityRadPerSec());
    inputs.indexerAppliedVolts = indexerAppliedVolts;
    inputs.indexerCurrentAmps = Math.abs(indexerSim.getCurrentDrawAmps());

    inputs.hotdogConnected = true;
    inputs.hotdogVelocityRps = Units.radiansToRotations(hotdogSim.getAngularVelocityRadPerSec());
    inputs.hotdogAppliedVolts = hotdogAppliedVolts;
    inputs.hotdogCurrentAmps = Math.abs(hotdogSim.getCurrentDrawAmps());
  }

  @Override
  public void setIndexerVelocity(double velocityRps) {
    indexerSetpointRps = velocityRps;
  }

  @Override
  public void setHotdogVelocity(double velocityRps) {
    hotdogSetpointRps = velocityRps;
  }
}
