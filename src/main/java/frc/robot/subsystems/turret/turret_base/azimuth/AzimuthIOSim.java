package frc.robot.subsystems.turret.turret_base.azimuth;

import static frc.robot.Constants.SubsystemConstants.Turret.azimuthGearRatio;
import static frc.robot.sim.MechanismSimConstants.LOOP_PERIOD_SECS;
import static frc.robot.sim.MechanismSimConstants.MAX_VOLTAGE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.sim.MechanismSimConstants;

public class AzimuthIOSim implements AzimuthIO {
  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              MechanismSimConstants.Turret.AZIMUTH_GEARBOX,
              MechanismSimConstants.Turret.AZIMUTH_MOI_KG_METERS_SQUARED,
              azimuthGearRatio),
          MechanismSimConstants.Turret.AZIMUTH_GEARBOX);
  private final PIDController controller =
      new PIDController(MechanismSimConstants.Turret.AZIMUTH_KP, 0.0, MechanismSimConstants.Turret.AZIMUTH_KD);

  private double targetAngleDeg = 0.0;
  private double sensorOffsetDeg = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(AzimuthIOInputs inputs) {
    appliedVolts =
        MathUtil.clamp(
            controller.calculate(sim.getAngularPositionRad(), Units.degreesToRadians(targetAngleDeg)),
            -MAX_VOLTAGE,
            MAX_VOLTAGE);
    sim.setInputVoltage(appliedVolts);
    sim.update(LOOP_PERIOD_SECS);

    inputs.connected = true;
    inputs.positionDeg = Units.radiansToDegrees(sim.getAngularPositionRad()) - sensorOffsetDeg;
    inputs.velocityDegPerSec = Units.radiansToDegrees(sim.getAngularVelocityRadPerSec());
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.zeroSwitchClosed = false;
  }

  @Override
  public void setTargetAngleDeg(double targetAngleDeg) {
    this.targetAngleDeg = targetAngleDeg;
  }

  @Override
  public void zeroPosition() {
    sensorOffsetDeg = Units.radiansToDegrees(sim.getAngularPositionRad());
    targetAngleDeg = 0.0;
  }
}
