package frc.robot.subsystems.turret;

import static frc.robot.Constants.SubsystemConstants.Turret.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.servohub.ServoChannel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.turret_base.Azimuth;
import frc.robot.subsystems.turret.turret_base.Flywheels;
import frc.robot.subsystems.turret.turret_base.Hood;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private static final double ANGLE_WRAP_DEGREES = 360.0;

  private final Hood hood;
  private final ServoChannel.ChannelId hoodChannel;
  private final Flywheels flywheels;
  private final Azimuth azimuth;
  private final String disableKey;
  private final double minAzimuthControlAngleDeg;
  private final double maxAzimuthControlAngleDeg;
  private BooleanSupplier bothTurretsReady = () -> false;

  private static boolean lockedIn = false;

  // private final Mechanism2d mechanism;
  // private final MechanismRoot2d mechRoot;
  // private final MechanismLigament2d turretLigament;

  public Turret(
      String name,
      int azimuthMotorId,
      TalonFXConfiguration azimuthConfig,
      double minAzimuthControlAngleDeg,
      double maxAzimuthControlAngleDeg,
      CANcoder azimuthEncoder,
      double startupCenterDeg,
      ServoChannel.ChannelId hoodChannelId,
      int flywheelID,
      TalonFXConfiguration flywheelConfig,
      int flywheelFollowerID,
      CANBus canbus) {
    setName(name);
    disableKey = name + "/Disable";
    SmartDashboard.setDefaultBoolean(disableKey, false);
    this.minAzimuthControlAngleDeg = minAzimuthControlAngleDeg;
    this.maxAzimuthControlAngleDeg = maxAzimuthControlAngleDeg;
    hoodChannel = hoodChannelId;
    hood = new Hood(hoodChannelId);
    flywheels = new Flywheels(flywheelID, flywheelConfig, flywheelFollowerID, canbus);
    azimuth =
        new Azimuth(
            azimuthMotorId,
            azimuthConfig,
            canbus,
            azimuthEncoder,
            startupCenterDeg,
            minAzimuthControlAngleDeg,
            maxAzimuthControlAngleDeg);
    if (isDisabledBySwitch()) {
      stop();
    }
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
    if (isDisabledBySwitch() || !bothTurretsReady.getAsBoolean()) {
      stop();
      return;
    }
    double referenceAngleDeg = azimuth.getWrapReferenceAngleDeg();
    if (!Double.isFinite(referenceAngleDeg)) {
      // Retry next scheduler tick without issuing targets from invalid startup feedback.
      stop();
      return;
    }
    double targetAzimuthDegrees = turretAngleRobot.getDegrees();
    double setpointDegrees =
        selectSafeSetpointDegrees(
            targetAzimuthDegrees,
            referenceAngleDeg,
            minAzimuthControlAngleDeg,
            maxAzimuthControlAngleDeg);
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

  public void setStartupInterlock(BooleanSupplier bothTurretsReady) {
    this.bothTurretsReady = bothTurretsReady;
  }

  public void initializeFromStartupWindow() {
    if (isDisabledBySwitch()) {
      stop();
      return;
    }
    azimuth.initializeFromStartupWindow();
  }

  /** Each dashboard switch blocks every actuator command for this turret in every mode. */
  @AutoLogOutput
  public boolean isDisabledBySwitch() {
    return SmartDashboard.getBoolean(disableKey, false);
  }

  @AutoLogOutput
  public boolean isStartupReady() {
    return azimuth.isStartupReady();
  }

  public void stop() {
    azimuth.stop();
    hood.stop();
    flywheels.stop();
  }

  /** Commanded hood position is not measured feedback; ball release data is measured externally. */
  public void recordShotLab(
      Pose2d pose,
      ChassisSpeeds velocity,
      double roll,
      double pitch,
      SimpleShotModel.Solution solution,
      GetAdjustedShot.ShootingParameters params,
      String mode) {
    String side = HOOD_SERVO_CHANNEL_1 == hoodChannel ? "Left" : "Right";
    String key = "ShotLab/" + side + "/";
    Logger.recordOutput(key + "Mode", mode);
    Logger.recordOutput(
        key + "Status",
        params == null ? mode : params.isValid() ? "Valid" : params.invalidReason());
    Logger.recordOutput(key + "Pose", pose);
    Logger.recordOutput(key + "FieldVelocity", velocity);
    Logger.recordOutput(key + "RollRad", roll);
    Logger.recordOutput(key + "PitchRad", pitch);
    Logger.recordOutput(key + "HoodCommandDeg", hood.getSetpointDeg());
    Logger.recordOutput(key + "WheelCommandRps", flywheels.getTargetRps());
    Logger.recordOutput(key + "MeasuredWheelRps", flywheels.getMeasuredVelocityRps());
    Logger.recordOutput(key + "FollowerWheelRps", flywheels.getFollowerVelocityRps());
    Logger.recordOutput(key + "MeasuredAzimuthDeg", azimuth.getMotorAngleDeg());
    Translation2d offset = side.equals("Left") ? robotToTurret1 : robotToTurret2;
    Translation2d offsetField = offset.rotateBy(pose.getRotation());
    Translation2d target =
        Constants.FieldConstants.getHubTranslation().minus(pose.getTranslation().plus(offsetField));
    double distance = target.getNorm();
    double vx = velocity.vxMetersPerSecond - velocity.omegaRadiansPerSecond * offsetField.getY();
    double vy = velocity.vyMetersPerSecond + velocity.omegaRadiansPerSecond * offsetField.getX();
    Logger.recordOutput(key + "BearingRad", target.getAngle().getRadians());
    Logger.recordOutput(key + "DistanceM", solution != null ? solution.distance() : distance);
    Logger.recordOutput(
        key + "RadialMps",
        solution != null
            ? solution.radial()
            : distance > 0 ? (vx * target.getX() + vy * target.getY()) / distance : 0);
    Logger.recordOutput(
        key + "TangentialMps",
        solution != null
            ? solution.tangential()
            : distance > 0 ? (-vx * target.getY() + vy * target.getX()) / distance : 0);
    Logger.recordOutput(key + "FlightTimeSec", solution != null ? solution.timeOfFlight() : 0.0);
  }

  @Override
  public void periodic() {
    azimuth.updateStartup(!isDisabledBySwitch());
    if (isDisabledBySwitch() || !bothTurretsReady.getAsBoolean()) {
      stop();
    }
    SmartDashboard.putBoolean(getName() + "/StartupReady", isStartupReady());
    SmartDashboard.putString(getName() + "/StartupStatus", azimuth.getStartupStatus());
    SmartDashboard.putNumber(getName() + "/StartupAngleDeg", azimuth.getStartupAngleDeg());
  }

  static double selectSafeSetpointDegrees(
      double targetAzimuthDegrees,
      double referenceAngleDeg,
      double minAzimuthControlAngleDeg,
      double maxAzimuthControlAngleDeg) {
    double referenceSetpointDegrees =
        MathUtil.clamp(referenceAngleDeg, minAzimuthControlAngleDeg, maxAzimuthControlAngleDeg);
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
