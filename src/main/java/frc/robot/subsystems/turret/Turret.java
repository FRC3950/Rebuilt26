package frc.robot.subsystems.turret;

import static frc.robot.Constants.SubsystemConstants.Turret.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.revrobotics.servohub.ServoChannel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.turret_base.Azimuth;
import frc.robot.subsystems.turret.turret_base.Flywheels;
import frc.robot.subsystems.turret.turret_base.Hood;
import org.littletonrobotics.junction.AutoLogOutput;

public class Turret extends SubsystemBase {
  private static final double ANGLE_WRAP_DEGREES = 360.0;

  private final Hood hood;
  private final Flywheels flywheels;
  private final Azimuth azimuth;
  private final double minAzimuthControlAngleDeg;
  private final double maxAzimuthControlAngleDeg;
  private final CANdi turretZeroingCandi;
  private final boolean usesCandiS1;

  private static boolean lockedIn = false;
  private boolean zeroSwitchClosedLastPoll = false;

  // private final Mechanism2d mechanism;
  // private final MechanismRoot2d mechRoot;
  // private final MechanismLigament2d turretLigament;

  public Turret(
      int azimuthMotorId,
      TalonFXConfiguration azimuthConfig,
      double minAzimuthControlAngleDeg,
      double maxAzimuthControlAngleDeg,
      ServoChannel.ChannelId hoodChannelId,
      int flywheelID,
      TalonFXConfiguration flywheelConfig,
      int flywheelFollowerID,
      CANBus canbus) {
    setName("Turret" + azimuthMotorId);
    this.minAzimuthControlAngleDeg = minAzimuthControlAngleDeg;
    this.maxAzimuthControlAngleDeg = maxAzimuthControlAngleDeg;
    usesCandiS1 = azimuthMotorId == frc.robot.Constants.SubsystemConstants.Turret.azimuthID;
    turretZeroingCandi =
        Constants.currentMode == Constants.Mode.REAL ? new CANdi(TURRET_CANDI_ID, canbus) : null;
    hood = new Hood(hoodChannelId);
    flywheels = new Flywheels(flywheelID, flywheelConfig, flywheelFollowerID, canbus);
    azimuth = new Azimuth(azimuthMotorId, azimuthConfig, canbus);
  }

  /** Returns the live turret pose with robot-relative translation and measured azimuth. */
  public Pose3d getRobotPose3d(Translation2d robotToTurret) {
    return new Pose3d(
        robotToTurret.getX(),
        robotToTurret.getY(),
        0.325,
        new Rotation3d(0.0, 0.0, Units.degreesToRadians(getVisualizationAngleDeg())));
  }

  private double getVisualizationAngleDeg() {
    return Constants.currentMode == Constants.Mode.SIM
        ? azimuth.getSetpointDeg()
        : azimuth.getMotorAngleDeg();
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

  public void runZeroAzimuthTarget(GetAdjustedShot.ShootingParameters params) {
    runSetpoints(new Rotation2d(-135), params.hoodAngleDeg(), params.flywheelSpeed());
  }

  public static void toggleTurretMode() {
    if (lockedIn) {
      lockedIn = false;
    } else {
      lockedIn = true;
    }
  }

  public static boolean getTargetingMode() {
    return lockedIn;
  }

  @AutoLogOutput
  public double getCommandedAzimuthDeg() {
    return azimuth.getSetpointDeg();
  }

  @AutoLogOutput
  public double getCommandedHoodAngleDeg() {
    return hood.getSetpointDeg();
  }

  @AutoLogOutput
  public double getCommandedFlywheelRps() {
    return flywheels.getTargetRps();
  }

  @AutoLogOutput
  public double getMeasuredAzimuthDeg() {
    return azimuth.getMeasuredAngleDeg();
  }

  @AutoLogOutput
  public double getMeasuredFlywheelRps() {
    return flywheels.getMeasuredVelocityRps();
  }

  @AutoLogOutput
  public double getMeasuredFlywheelFollowerRps() {
    return flywheels.getFollowerVelocityRps();
  }

  @AutoLogOutput
  public boolean isTargetingLocked() {
    return getTargetingMode();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() && turretZeroingCandi != null) {
      updateDisabledZeroing();
    } else {
      zeroSwitchClosedLastPoll = false;
    }
  }

  private void updateDisabledZeroing() {
    boolean zeroSwitchClosed =
        usesCandiS1
            ? Boolean.TRUE.equals(turretZeroingCandi.getS1Closed().getValue())
            : Boolean.TRUE.equals(turretZeroingCandi.getS2Closed().getValue());
    if (zeroSwitchClosed && !zeroSwitchClosedLastPoll) {
      azimuth.zeroPosition();
    }
    zeroSwitchClosedLastPoll = zeroSwitchClosed;
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
