package frc.robot.subsystems.turret;

import static frc.robot.Constants.SubsystemConstants.Turret.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
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
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;

public class Turret extends SubsystemBase {
  private static ServoHub hoodServoHub;

  private final ServoChannel hoodServo;
  private final TalonFX flywheel;
  private final TalonFX flywheelFollower;
  private final TalonFX azimuth;

  private final Mechanism2d mechanism;
  private final MechanismRoot2d mechRoot;
  private final MechanismLigament2d turretLigament;

  // Controls
  private final MotionMagicVoltage azimuthControl = new MotionMagicVoltage(0);
  private final BangBangController flywheelControl =
      new BangBangController(speedTolerance); // Tolerance 50 RPS (placeholder)

  private double lastAzimuthSetpointDeg = 0.0;
  private double lastHoodSetpointDeg = minHoodAngle;
  private double lastFlywheelSetpointRps = 0.0;
  private double hoodPositionDeg = minHoodAngle;
  private double hoodVelocityDegPerSec = 0.0;

  public Turret(
      int azimuthID,
      TalonFXConfiguration azimuthConfig,
      ServoChannel.ChannelId hoodChannelId,
      int flywheelID,
      TalonFXConfiguration flywheelConfig,
      int flywheelFollowerID,
      CANBus canbus) {
    hoodServo = getConfiguredHoodServoHub().getServoChannel(hoodChannelId);
    flywheel = new TalonFX(flywheelID, canbus);
    flywheelFollower = new TalonFX(flywheelFollowerID, canbus);
    azimuth = new TalonFX(azimuthID, canbus);

    configureMotors(azimuthConfig, flywheelConfig, flywheelID);
    initializeHoodAtMinimum();

    // Mechanism 2D
    mechanism = new Mechanism2d(3, 3);
    mechRoot = mechanism.getRoot("TurretRoot", 1.5, 1.5);
    turretLigament =
        mechRoot.append(new MechanismLigament2d("Turret", 0.5, 0, 6, new Color8Bit(Color.kBlue)));
    SmartDashboard.putData("Turret Mechanism", mechanism);
  }

  private void configureMotors(
      TalonFXConfiguration azimuthConfig,
      TalonFXConfiguration flywheelConfig,
      int flywheelID) {
    // Azimuth Motor Config
    azimuth.getConfigurator().apply(azimuthConfig);

    // Flywheel Motor Config
    flywheel.getConfigurator().apply(flywheelConfig);

    // Flywheel Follower Config
    flywheelFollower.setControl(new Follower(flywheelID, MotorAlignmentValue.Opposed));

    zeroEncoders();
  }

  public void zeroEncoders() {
    azimuth.setPosition(0);
    flywheel.setPosition(0);
  }

  @Override
  public void periodic() {
    double azimuthRotations = azimuth.getPosition().getValueAsDouble();
    double currentAzimuthDegrees = Units.rotationsToDegrees(azimuthRotations / azimuthGearRatio);
    turretLigament.setAngle(currentAzimuthDegrees);
    updateHoodTelemetry();
  }

  public void runSetpoints(Rotation2d turretAngleRobot, double hoodAngleDeg, double flywheelSpeed) {
    double targetAzimuthDegrees = turretAngleRobot.getDegrees();

    double azimuthRotations = azimuth.getPosition().getValueAsDouble();
    double currentAzimuthDegrees = Units.rotationsToDegrees(azimuthRotations / azimuthGearRatio);

    double deltaDegrees =
        MathUtil.inputModulus(
            targetAzimuthDegrees - currentAzimuthDegrees, minAzimuthAngle, maxAzimuthAngle);
    double setpointDegrees = currentAzimuthDegrees + deltaDegrees;
    double clampedHoodAngleDeg = MathUtil.clamp(hoodAngleDeg, minHoodAngle, maxHoodAngle);

    lastAzimuthSetpointDeg = setpointDegrees;
    lastHoodSetpointDeg = clampedHoodAngleDeg;
    lastFlywheelSetpointRps = flywheelSpeed;

    azimuth.setControl(
        azimuthControl.withPosition(Units.degreesToRotations(setpointDegrees) * azimuthGearRatio));
    hoodServo.setPulseWidth(hoodAngleToPulseWidthUs(clampedHoodAngleDeg));

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
    return hoodPositionDeg;
  }

  @AutoLogOutput(key = "Turret/Hood/VelocityDegPerSec")
  public double getHoodVelocityDegPerSec() {
    return hoodVelocityDegPerSec;
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

  private static synchronized ServoHub getConfiguredHoodServoHub() {
    if (hoodServoHub == null) {
      hoodServoHub = new ServoHub(HOOD_SERVO_HUB_CAN_ID);

      ServoHubConfig hubConfig = new ServoHubConfig();
      ServoChannelConfig turret1Config =
          new ServoChannelConfig(HOOD_SERVO_CHANNEL_1)
              .pulseRange(
                  HOOD_SERVO_MIN_PULSE_US, HOOD_SERVO_CENTER_PULSE_US, HOOD_SERVO_MAX_PULSE_US)
              .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
      ServoChannelConfig turret2Config =
          new ServoChannelConfig(HOOD_SERVO_CHANNEL_2)
              .pulseRange(
                  HOOD_SERVO_MIN_PULSE_US, HOOD_SERVO_CENTER_PULSE_US, HOOD_SERVO_MAX_PULSE_US)
              .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);

      hubConfig.apply(HOOD_SERVO_CHANNEL_1, turret1Config);
      hubConfig.apply(HOOD_SERVO_CHANNEL_2, turret2Config);
      hoodServoHub.configure(hubConfig, ResetMode.kNoResetSafeParameters);
    }
    return hoodServoHub;
  }

  private void initializeHoodAtMinimum() {
    lastHoodSetpointDeg = minHoodAngle;
    hoodPositionDeg = minHoodAngle;
    hoodVelocityDegPerSec = 0.0;

    hoodServo.setEnabled(true);
    hoodServo.setPowered(true);
    hoodServo.setPulseWidth(HOOD_SERVO_MIN_PULSE_US);
  }

  private void updateHoodTelemetry() {
    double measuredHoodAngleDeg = pulseWidthUsToHoodAngleDeg(hoodServo.getPulseWidth());
    if (!Double.isFinite(measuredHoodAngleDeg)) {
      measuredHoodAngleDeg = lastHoodSetpointDeg;
    }

    hoodVelocityDegPerSec = (measuredHoodAngleDeg - hoodPositionDeg) / Constants.loopPeriodSecs;
    hoodPositionDeg = measuredHoodAngleDeg;
  }

  private static int hoodAngleToPulseWidthUs(double hoodAngleDeg) {
    double clampedHoodAngleDeg = MathUtil.clamp(hoodAngleDeg, minHoodAngle, maxHoodAngle);
    double hoodAngleRangeDeg = maxHoodAngle - minHoodAngle;
    int servoPulseRangeUs = HOOD_SERVO_MAX_PULSE_US - HOOD_SERVO_MIN_PULSE_US;
    if (hoodAngleRangeDeg <= 0.0 || servoPulseRangeUs <= 0) {
      return HOOD_SERVO_MIN_PULSE_US;
    }

    double t = (clampedHoodAngleDeg - minHoodAngle) / hoodAngleRangeDeg;
    int pulseUs = (int) Math.round(HOOD_SERVO_MIN_PULSE_US + t * servoPulseRangeUs);
    return Math.max(HOOD_SERVO_MIN_PULSE_US, Math.min(HOOD_SERVO_MAX_PULSE_US, pulseUs));
  }

  private static double pulseWidthUsToHoodAngleDeg(int pulseWidthUs) {
    double hoodAngleRangeDeg = maxHoodAngle - minHoodAngle;
    int servoPulseRangeUs = HOOD_SERVO_MAX_PULSE_US - HOOD_SERVO_MIN_PULSE_US;
    if (hoodAngleRangeDeg <= 0.0 || servoPulseRangeUs <= 0) {
      return minHoodAngle;
    }

    int clampedPulseUs =
        Math.max(HOOD_SERVO_MIN_PULSE_US, Math.min(HOOD_SERVO_MAX_PULSE_US, pulseWidthUs));
    double t = (double) (clampedPulseUs - HOOD_SERVO_MIN_PULSE_US) / servoPulseRangeUs;
    return minHoodAngle + t * hoodAngleRangeDeg;
  }
}
