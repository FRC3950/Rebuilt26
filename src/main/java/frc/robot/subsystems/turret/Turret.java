// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import static frc.robot.Constants.SubsystemConstants.Turret.*;
import static frc.robot.Constants.FieldConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class Turret extends SubsystemBase {
  private final TalonFX hood;
  private final TalonFX flywheel;
  private final TalonFX flywheelFollower;
  private final TalonFX azimuth;

  private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();

  private final Supplier<Pose2d> poseSupplier;

  private final Mechanism2d mechanism;
  private final MechanismRoot2d mechRoot;
  private final MechanismLigament2d turretLigament;

  // Controls
  private final MotionMagicVoltage azimuthControl = new MotionMagicVoltage(0);
  private final PositionVoltage hoodControl = new PositionVoltage(0);
  private final VelocityVoltage flywheelControl = new VelocityVoltage(0);

  public Turret(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;

    hood = new TalonFX(hoodID, turretCanbus);
    flywheel = new TalonFX(flywheelID, turretCanbus);
    flywheelFollower = new TalonFX(flywheelFollowerID, turretCanbus);
    azimuth = new TalonFX(azimuthID, turretCanbus);

    configureMotors();
    fillInterpolationMaps();

    // Mechanism 2D
    mechanism = new Mechanism2d(3, 3);
    mechRoot = mechanism.getRoot("TurretRoot", 1.5, 1.5);
    turretLigament = mechRoot.append(
        new MechanismLigament2d("Turret", 0.5, 0, 6, new Color8Bit(Color.kBlue)));
    SmartDashboard.putData("Turret Mechanism", mechanism);
  }

  private void configureMotors() {
    //Azimuth Motor Config
    TalonFXConfiguration azimuthConfig = new TalonFXConfiguration();
    azimuthConfig.Slot0.kP = azimuthKP;
    azimuthConfig.Slot0.kI = azimuthKI;
    azimuthConfig.Slot0.kD = azimuthKD;
    azimuthConfig.Slot0.kV = azimuthKV;
    azimuthConfig.Slot0.kS = azimuthKS;
    azimuthConfig.Slot0.kA = azimuthKA;
    azimuthConfig.MotionMagic.MotionMagicCruiseVelocity = azimuthMaxVelocity / 360.0 * azimuthGearRatio; // Rotations/s
    azimuthConfig.MotionMagic.MotionMagicAcceleration = azimuthMaxAcceleration / 360.0 * azimuthGearRatio; // Rotations/s^2
    azimuthConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    azimuth.getConfigurator().apply(azimuthConfig);
    
    //Hood Motor Config
    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.Slot0.kP = hoodKP;
    hoodConfig.Slot0.kI = hoodKI;
    hoodConfig.Slot0.kD = hoodKD;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hood.getConfigurator().apply(hoodConfig);

    //Flywheel Motor Config
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.Slot0.kP = flywheelKP;
    flywheelConfig.Slot0.kI = flywheelKI;
    flywheelConfig.Slot0.kD = flywheelKD;
    flywheelConfig.Slot0.kV = flywheelKV;
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheel.getConfigurator().apply(flywheelConfig);

    //Flywheel Follower Config
    flywheelFollower.setControl(new Follower(flywheelID, MotorAlignmentValue.Opposed));

    zeroEncoders();
  }

  private void fillInterpolationMaps() {
    for (int i = 0; i < shootingDistances.length; i++) {
      hoodMap.put(shootingDistances[i], hoodAngles[i]);
      flywheelMap.put(shootingDistances[i], flywheelSpeeds[i]);
    }
  }

  public void zeroEncoders() {
    azimuth.setPosition(0);
    hood.setPosition(0);
    flywheel.setPosition(0);
  }

  @Override
  public void periodic() {
    Pose2d robotPose = poseSupplier.get();
    Translation2d robotTranslation = robotPose.getTranslation();
    double distanceToHub = robotTranslation.getDistance(hubTranslation);

    // Calculate interpolations
    double targetHoodAngle = hoodMap.get(distanceToHub);
    double targetFlywheelSpeed = flywheelMap.get(distanceToHub);

    // Calculate Turret Angle
    // Angle from robot to hub
    Rotation2d angleToHub = new Rotation2d(
        hubTranslation.getX() - robotTranslation.getX(),
        hubTranslation.getY() - robotTranslation.getY());

    Rotation2d targetTurretRotation = angleToHub.minus(robotPose.getRotation());

    double targetAzimuthDegrees = targetTurretRotation.getDegrees();

    // Angle Wrapping for continuous rotation
    // Get current position in degrees
    double carRotationRotations = azimuth.getPosition().getValueAsDouble();
    double currentAzimuthDegrees = Units.rotationsToDegrees(carRotationRotations / azimuthGearRatio);

    //logic to turn the shortest way to the setpoint and not
    double deltaDegrees = MathUtil.inputModulus(targetAzimuthDegrees - currentAzimuthDegrees, -180, 180);
    double setpointDegrees = currentAzimuthDegrees + deltaDegrees;

    //Control all motors autonomously based on distances
    azimuth.setControl(azimuthControl.withPosition(Units.degreesToRotations(setpointDegrees) * azimuthGearRatio));
    hood.setControl(hoodControl.withPosition(Units.degreesToRotations(targetHoodAngle) * hoodGearRatio));
    flywheel.setControl(flywheelControl.withVelocity(targetFlywheelSpeed * flywheelGearRatio));

    // Visualizer
    turretLigament.setAngle(Units.rotationsToDegrees(carRotationRotations));                                                                           // rotation
  }
}
