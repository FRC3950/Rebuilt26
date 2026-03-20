package frc.robot.subsystems.turret;

import static frc.robot.Constants.SubsystemConstants.Turret.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.servohub.ServoChannel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.turret_base.Azimuth;
import frc.robot.subsystems.turret.turret_base.Flywheels;
import frc.robot.subsystems.turret.turret_base.Hood;

public class Turret extends SubsystemBase {
  private static final double ANGLE_WRAP_DEGREES = 360.0;

  private final Hood hood;
  private final Flywheels flywheels;
  private final Azimuth azimuth;
  private final double minAzimuthControlAngleDeg;
  private final double maxAzimuthControlAngleDeg;

  // private final Mechanism2d mechanism;
  // private final MechanismRoot2d mechRoot;
  // private final MechanismLigament2d turretLigament;

  public Turret(
      int azimuthID,
      TalonFXConfiguration azimuthConfig,
      double minAzimuthControlAngleDeg,
      double maxAzimuthControlAngleDeg,
      ServoChannel.ChannelId hoodChannelId,
      int flywheelID,
      TalonFXConfiguration flywheelConfig,
      int flywheelFollowerID,
      CANBus canbus) {
    this.minAzimuthControlAngleDeg = minAzimuthControlAngleDeg;
    this.maxAzimuthControlAngleDeg = maxAzimuthControlAngleDeg;
    hood = new Hood(hoodChannelId);
    flywheels = new Flywheels(flywheelID, flywheelConfig, flywheelFollowerID, canbus);
    azimuth = new Azimuth(azimuthID, azimuthConfig, canbus);
  }

  public void runSetpoints(Rotation2d turretAngleRobot, double hoodAngleDeg, double flywheelSpeed) {
    double targetAzimuthDegrees = turretAngleRobot.getDegrees();
    double setpointDegrees = selectSafeSetpointDegrees(targetAzimuthDegrees);
    double clampedHoodAngleDeg = MathUtil.clamp(hoodAngleDeg, minHoodAngle, maxHoodAngle);

    azimuth.setTargetAngleDeg(setpointDegrees);
    hood.setAngleDeg(clampedHoodAngleDeg);
    flywheels.setTargetRps(flywheelSpeed);
  }

  public void runAutoTarget(GetAdjustedShot.ShootingParameters params) {
    runSetpoints(params.turretAngle(), params.hoodAngleDeg(), params.flywheelSpeed());
  }

  private double selectSafeSetpointDegrees(double targetAzimuthDegrees) {
    double referenceSetpointDegrees =
        MathUtil.clamp(
            azimuth.getSetpointDeg(), minAzimuthControlAngleDeg, maxAzimuthControlAngleDeg);
    double bestCandidateDegrees = Double.NaN;
    double bestErrorDegrees = Double.POSITIVE_INFINITY;

    int minWrapIndex =
        (int) Math.ceil((minAzimuthControlAngleDeg - targetAzimuthDegrees) / ANGLE_WRAP_DEGREES);
    int maxWrapIndex =
        (int) Math.floor((maxAzimuthControlAngleDeg - targetAzimuthDegrees) / ANGLE_WRAP_DEGREES);

    for (int wrapIndex = minWrapIndex; wrapIndex <= maxWrapIndex; wrapIndex++) {
      double candidateDegrees = targetAzimuthDegrees + ANGLE_WRAP_DEGREES * wrapIndex;
      double errorDegrees = Math.abs(candidateDegrees - referenceSetpointDegrees);
      if (errorDegrees < bestErrorDegrees) {
        bestCandidateDegrees = candidateDegrees;
        bestErrorDegrees = errorDegrees;
      }
    }

    if (Double.isNaN(bestCandidateDegrees)) {
      return MathUtil.clamp(
          targetAzimuthDegrees, minAzimuthControlAngleDeg, maxAzimuthControlAngleDeg);
    }
    return bestCandidateDegrees;
  }
}
