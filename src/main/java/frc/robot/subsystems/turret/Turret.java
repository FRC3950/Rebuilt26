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
  private final Hood hood;
  private final Flywheels flywheels;
  private final Azimuth azimuth;
  private boolean hoodSafetyForcedDown = false;

  // private final Mechanism2d mechanism;
  // private final MechanismRoot2d mechRoot;
  // private final MechanismLigament2d turretLigament;

  public Turret(
      int azimuthID,
      TalonFXConfiguration azimuthConfig,
      ServoChannel.ChannelId hoodChannelId,
      int flywheelID,
      TalonFXConfiguration flywheelConfig,
      int flywheelFollowerID,
      CANBus canbus) {
    hood = new Hood(hoodChannelId);
    flywheels = new Flywheels(flywheelID, flywheelConfig, flywheelFollowerID, canbus);
    azimuth = new Azimuth(azimuthID, azimuthConfig, canbus);
  }

  public void runSetpoints(Rotation2d turretAngleRobot, double hoodAngleDeg, double flywheelSpeed) {
    setAzimuthTarget(turretAngleRobot);
    applyHoodAngle(hoodAngleDeg);
    flywheels.setTargetRps(flywheelSpeed);
  }

  public void trackTargetOnly(Rotation2d turretAngleRobot) {
    setAzimuthTarget(turretAngleRobot);
    hood.hoodDown();
    flywheels.setTargetRps(0.0);
  }

  private void setAzimuthTarget(Rotation2d turretAngleRobot) {
    double targetAzimuthDegrees = turretAngleRobot.getDegrees();
    double currentAzimuthDegrees = azimuth.getMotorAngleDeg();

    double deltaDegrees =
        MathUtil.inputModulus(
            targetAzimuthDegrees - currentAzimuthDegrees, minAzimuthAngle, maxAzimuthAngle);
    azimuth.setTargetAngleDeg(currentAzimuthDegrees + deltaDegrees);
  }

  private void applyHoodAngle(double hoodAngleDeg) {
    double clampedHoodAngleDeg = MathUtil.clamp(hoodAngleDeg, minHoodAngle, maxHoodAngle);
    if (hoodSafetyForcedDown) {
      hood.hoodDown();
    } else {
      hood.setAngleDeg(clampedHoodAngleDeg);
    }
  }

  public void setHoodSafetyForcedDown(boolean enabled) {
    hoodSafetyForcedDown = enabled;
  }
}
