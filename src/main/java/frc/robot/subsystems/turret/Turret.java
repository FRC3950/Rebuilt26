package frc.robot.subsystems.turret;

import static frc.robot.Constants.SubsystemConstants.Turret.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.servohub.ServoChannel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.turret_base.Azimuth;
import frc.robot.subsystems.turret.turret_base.Azimuth.LimitSwitchChannel;
import frc.robot.subsystems.turret.turret_base.Flywheels;
import frc.robot.subsystems.turret.turret_base.Hood;
import org.littletonrobotics.junction.AutoLogOutput;

public class Turret extends SubsystemBase {
  private final Hood hood;
  private final Flywheels flywheels;
  private final Azimuth azimuth;
  private boolean hoodSafetyForcedDown = false;

  private final Mechanism2d mechanism;
  private final MechanismRoot2d mechRoot;
  private final MechanismLigament2d turretLigament;

  public Turret(
      int azimuthID,
      TalonFXConfiguration azimuthConfig,
      LimitSwitchChannel limitSwitchChannel,
      ServoChannel.ChannelId hoodChannelId,
      int flywheelID,
      TalonFXConfiguration flywheelConfig,
      int flywheelFollowerID,
      CANBus canbus) {
    hood = new Hood(hoodChannelId);
    flywheels = new Flywheels(flywheelID, flywheelConfig, flywheelFollowerID, canbus);
    azimuth = new Azimuth(azimuthID, azimuthConfig, canbus, limitSwitchChannel);
    azimuth.zeroMotorPosition();
    flywheels.zeroPosition();

    mechanism = new Mechanism2d(3, 3);
    mechRoot = mechanism.getRoot("TurretRoot", 1.5, 1.5);
    turretLigament =
        mechRoot.append(new MechanismLigament2d("Turret", 0.5, 0, 6, new Color8Bit(Color.kBlue)));
    SmartDashboard.putData("Turret Mechanism", mechanism);
  }

  public void zeroEncoders() {
    azimuth.zeroMotorPosition();
    flywheels.zeroPosition();
  }

  @Override
  public void periodic() {
    turretLigament.setAngle(azimuth.getMotorAngleDeg());
    azimuth.syncMotorPositionToLimitSwitchIfTriggered();
    hood.periodic();
  }

  public void runSetpoints(Rotation2d turretAngleRobot, double hoodAngleDeg, double flywheelSpeed) {
    double targetAzimuthDegrees = turretAngleRobot.getDegrees();
    double currentAzimuthDegrees = azimuth.getMotorAngleDeg();

    double deltaDegrees =
        MathUtil.inputModulus(
            targetAzimuthDegrees - currentAzimuthDegrees, minAzimuthAngle, maxAzimuthAngle);
    double setpointDegrees = currentAzimuthDegrees + deltaDegrees;
    double clampedHoodAngleDeg = MathUtil.clamp(hoodAngleDeg, minHoodAngle, maxHoodAngle);

    azimuth.setTargetAngleDeg(setpointDegrees);
    if (hoodSafetyForcedDown) {
      hood.hoodDown();
    } else {
      hood.setAngleDeg(clampedHoodAngleDeg);
    }
    flywheels.setTargetRps(flywheelSpeed);
  }

  public void setHoodSafetyForcedDown(boolean enabled) {
    hoodSafetyForcedDown = enabled;
  }

  public void runAutoTarget(GetAdjustedShot.ShootingParameters params) {
    runSetpoints(params.turretAngle(), params.hoodAngleDeg(), params.flywheelSpeed());
  }

  public static boolean isManualStickActive(double stickX, double stickY, double deadband) {
    return Math.hypot(stickX, stickY) > deadband;
  }

  public static Rotation2d getFieldHeadingFromStick(double stickX, double stickY) {
    // Operator "forward" is fixed to +X in the WPILib field frame.
    double x = -stickY;
    double y = -stickX;
    return new Rotation2d(Math.atan2(y, x));
  }

  public void runManualFieldHeading(Rotation2d desiredFieldHeading, Rotation2d robotHeading) {
    Rotation2d turretRobotHeading = desiredFieldHeading.minus(robotHeading);
    runSetpoints(turretRobotHeading, getHoodSetpointDeg(), getFlywheelSetpointRps());
  }

  public void holdCurrentPositionWithStoppedFlywheel() {
    runSetpoints(Rotation2d.fromDegrees(getAzimuthPositionDeg()), getHoodPositionDeg(), 0.0);
  }

  public void stopFlywheel() {
    flywheels.stop();
  }

  @AutoLogOutput(key = "Turret/Mechanism")
  public Mechanism2d getMechanism() {
    return mechanism;
  }

  @AutoLogOutput(key = "Turret/Azimuth/PositionDeg")
  public double getAzimuthPositionDeg() {
    return azimuth.getMotorAngleDeg();
  }

  @AutoLogOutput(key = "Turret/Azimuth/VelocityDegPerSec")
  public double getAzimuthVelocityDegPerSec() {
    return azimuth.getMotorVelocityDegPerSec();
  }

  @AutoLogOutput(key = "Turret/Azimuth/SetpointDeg")
  public double getAzimuthSetpointDeg() {
    return azimuth.getSetpointDeg();
  }

  @AutoLogOutput(key = "Turret/Hood/PositionDeg")
  public double getHoodPositionDeg() {
    return hood.getPositionDeg();
  }

  @AutoLogOutput(key = "Turret/Hood/SetpointDeg")
  public double getHoodSetpointDeg() {
    return hood.getSetpointDeg();
  }

  @AutoLogOutput(key = "Turret/Flywheel/VelocityRps")
  public double getFlywheelVelocityRps() {
    return flywheels.getVelocityRps();
  }

  @AutoLogOutput(key = "Turret/Flywheel/SetpointRps")
  public double getFlywheelSetpointRps() {
    return flywheels.getSetpointRps();
  }

  @AutoLogOutput(key = "Turret/FlywheelFollower/VelocityRps")
  public double getFlywheelFollowerVelocityRps() {
    return flywheels.getFollowerVelocityRps();
  }
}
