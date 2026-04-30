package frc.robot.subsystems.turret.turret_base.hood;

import static frc.robot.Constants.SubsystemConstants.Turret.HOOD_SERVO_MAX_PULSE_US;
import static frc.robot.Constants.SubsystemConstants.Turret.HOOD_SERVO_MIN_PULSE_US;
import static frc.robot.sim.MechanismSimConstants.LOOP_PERIOD_SECS;

import edu.wpi.first.math.MathUtil;
import frc.robot.sim.MechanismSimConstants;

public class HoodIOSim implements HoodIO {
  private double targetAngleDeg = MechanismSimConstants.Turret.HOOD_MIN_ANGLE_DEG;
  private double positionDeg = MechanismSimConstants.Turret.HOOD_MIN_ANGLE_DEG;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double maxStep = MechanismSimConstants.Turret.HOOD_MAX_VELOCITY_DEG_PER_SEC * LOOP_PERIOD_SECS;
    positionDeg += MathUtil.clamp(targetAngleDeg - positionDeg, -maxStep, maxStep);

    inputs.connected = true;
    inputs.positionDeg = positionDeg;
    inputs.pulseWidthUs =
        MathUtil.interpolate(
            HOOD_SERVO_MIN_PULSE_US,
            HOOD_SERVO_MAX_PULSE_US,
            (positionDeg - MechanismSimConstants.Turret.HOOD_MIN_ANGLE_DEG)
                / (MechanismSimConstants.Turret.HOOD_MAX_ANGLE_DEG
                    - MechanismSimConstants.Turret.HOOD_MIN_ANGLE_DEG));
  }

  @Override
  public void setAngleDeg(double hoodAngleDeg) {
    targetAngleDeg =
        MathUtil.clamp(
            hoodAngleDeg,
            MechanismSimConstants.Turret.HOOD_MIN_ANGLE_DEG,
            MechanismSimConstants.Turret.HOOD_MAX_ANGLE_DEG);
  }
}
