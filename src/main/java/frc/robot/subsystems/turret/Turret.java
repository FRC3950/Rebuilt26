package frc.robot.subsystems.turret;

import static frc.robot.Constants.SubsystemConstants.Turret.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Turret extends SubsystemBase {
  private final TalonFX hood;
  private final TalonFX flywheel;
  private final TalonFX flywheelFollower;
  private final TalonFX azimuth;

  private final Mechanism2d mechanism;
  private final MechanismRoot2d mechRoot;
  private final MechanismLigament2d turretLigament;

  // Controls
  private final MotionMagicVoltage azimuthControl = new MotionMagicVoltage(0);
  private final PositionVoltage hoodControl = new PositionVoltage(0);
  private final BangBangController flywheelControl =
      new BangBangController(speedTolerance); // Tolerance 50 RPS (placeholder)

  private double lastAzimuthSetpointDeg = 0.0;
  private double lastHoodSetpointDeg = 0.0;
  private double lastFlywheelSetpointRps = 0.0;

  public Turret(
      int azimuthID,
      TalonFXConfiguration azimuthConfig,
      int hoodID,
      TalonFXConfiguration hoodConfig,
      int flywheelID,
      TalonFXConfiguration flywheelConfig,
      int flywheelFollowerID,
      CANBus canbus) {
    hood = new TalonFX(hoodID, canbus);
    flywheel = new TalonFX(flywheelID, canbus);
    flywheelFollower = new TalonFX(flywheelFollowerID, canbus);
    azimuth = new TalonFX(azimuthID, canbus);

    configureMotors(azimuthConfig, hoodConfig, flywheelConfig, flywheelID);

    // Mechanism 2D
    mechanism = new Mechanism2d(3, 3);
    mechRoot = mechanism.getRoot("TurretRoot", 1.5, 1.5);
    turretLigament =
        mechRoot.append(new MechanismLigament2d("Turret", 0.5, 0, 6, new Color8Bit(Color.kBlue)));
    SmartDashboard.putData("Turret Mechanism", mechanism);
  }

  private void configureMotors(
      TalonFXConfiguration azimuthConfig,
      TalonFXConfiguration hoodConfig,
      TalonFXConfiguration flywheelConfig,
      int flywheelID) {
    // Azimuth Motor Config
    azimuth.getConfigurator().apply(azimuthConfig);

    // Hood Motor Config
    hood.getConfigurator().apply(hoodConfig);

    // Flywheel Motor Config
    flywheel.getConfigurator().apply(flywheelConfig);

    // Flywheel Follower Config
    flywheelFollower.setControl(new Follower(flywheelID, MotorAlignmentValue.Opposed));

    zeroEncoders();
  }

  public void zeroEncoders() {
    azimuth.setPosition(0);
    hood.setPosition(0);
    flywheel.setPosition(0);
  }

  @Override
  public void periodic() {
    double azimuthRotations = azimuth.getPosition().getValueAsDouble();
    double currentAzimuthDegrees = Units.rotationsToDegrees(azimuthRotations / azimuthGearRatio);
    turretLigament.setAngle(currentAzimuthDegrees);
  }

  public void runSetpoints(Rotation2d turretAngleRobot, double hoodAngleDeg, double flywheelSpeed) {
    double targetAzimuthDegrees = turretAngleRobot.getDegrees();

    double azimuthRotations = azimuth.getPosition().getValueAsDouble();
    double currentAzimuthDegrees = Units.rotationsToDegrees(azimuthRotations / azimuthGearRatio);

    double deltaDegrees =
        MathUtil.inputModulus(
            targetAzimuthDegrees - currentAzimuthDegrees, minAzimuthAngle, maxAzimuthAngle);
    double setpointDegrees = currentAzimuthDegrees + deltaDegrees;
    lastAzimuthSetpointDeg = setpointDegrees;
    lastHoodSetpointDeg = hoodAngleDeg;
    lastFlywheelSetpointRps = flywheelSpeed;

    azimuth.setControl(
        azimuthControl.withPosition(Units.degreesToRotations(setpointDegrees) * azimuthGearRatio));
    hood.setControl(
        hoodControl.withPosition(Units.degreesToRotations(hoodAngleDeg) * hoodGearRatio));

    double currentVel = flywheel.getVelocity().getValueAsDouble();
    double targetVel = flywheelSpeed * flywheelGearRatio;

    // BangBangController returns 1 if setpoint > measurement, 0 if setpoint <
    // measurement.

    double speed = flywheelControl.calculate(currentVel, targetVel);
    flywheel.set(speed);
  }

  public void runAutoTarget(GetAdjustedShot.ShootingParameters params) {
    runSetpoints(
        params.turretAngle(), Units.radiansToDegrees(params.hoodAngle()), params.flywheelSpeed());
  }

  public void stopFlywheel() {
    flywheel.set(0);
    lastFlywheelSetpointRps = 0.0;
  }

  @AutoLogOutput(key = "Turret/Mechanism")
  public Mechanism2d getMechanism() {
    return mechanism;
  }

  @AutoLogOutput(key = "Turret/Azimuth/PositionDeg")
  public double getAzimuthPositionDeg() {
    return Units.rotationsToDegrees(azimuth.getPosition().getValueAsDouble() / azimuthGearRatio);
  }

  @AutoLogOutput(key = "Turret/Azimuth/VelocityDegPerSec")
  public double getAzimuthVelocityDegPerSec() {
    return Units.rotationsToDegrees(azimuth.getVelocity().getValueAsDouble() / azimuthGearRatio);
  }

  @AutoLogOutput(key = "Turret/Azimuth/SetpointDeg")
  public double getAzimuthSetpointDeg() {
    return lastAzimuthSetpointDeg;
  }

  @AutoLogOutput(key = "Turret/Hood/PositionDeg")
  public double getHoodPositionDeg() {
    return Units.rotationsToDegrees(hood.getPosition().getValueAsDouble() / hoodGearRatio);
  }

  @AutoLogOutput(key = "Turret/Hood/VelocityDegPerSec")
  public double getHoodVelocityDegPerSec() {
    return Units.rotationsToDegrees(hood.getVelocity().getValueAsDouble() / hoodGearRatio);
  }

  @AutoLogOutput(key = "Turret/Hood/SetpointDeg")
  public double getHoodSetpointDeg() {
    return lastHoodSetpointDeg;
  }

  @AutoLogOutput(key = "Turret/Flywheel/PositionRot")
  public double getFlywheelPositionRot() {
    return flywheel.getPosition().getValueAsDouble();
  }

  @AutoLogOutput(key = "Turret/Flywheel/VelocityRps")
  public double getFlywheelVelocityRps() {
    return flywheel.getVelocity().getValueAsDouble();
  }

  @AutoLogOutput(key = "Turret/Flywheel/SetpointRps")
  public double getFlywheelSetpointRps() {
    return lastFlywheelSetpointRps;
  }

  @AutoLogOutput(key = "Turret/FlywheelFollower/PositionRot")
  public double getFlywheelFollowerPositionRot() {
    return flywheelFollower.getPosition().getValueAsDouble();
  }

  @AutoLogOutput(key = "Turret/FlywheelFollower/VelocityRps")
  public double getFlywheelFollowerVelocityRps() {
    return flywheelFollower.getVelocity().getValueAsDouble();
  }
}
