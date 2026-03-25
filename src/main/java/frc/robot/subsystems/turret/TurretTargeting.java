package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Default turret targeting command.
 *
 * <p>Calculates turret azimuth from the robot's current pose and optionally applies manual hood and
 * flywheel setpoints.
 */
public class TurretTargeting extends Command {
  public enum OutputMode {
    AZIMUTH_ONLY,
    MANUAL_SETPOINTS_WHEN_ENABLED
  }

  private final Turret turret;
  private final Drive drive;
  private final Translation2d robotToTurret;
  private final Supplier<Translation2d> targetOverrideSupplier;
  private final OutputMode outputMode;
  private final DoubleSupplier hoodAngleDegSupplier;
  private final DoubleSupplier flywheelRpsSupplier;
  private final BooleanSupplier runSetpointsSupplier;
  private final GetAdjustedShot shotCalc = new GetAdjustedShot();

  public TurretTargeting(Turret turret, Drive drive, Translation2d robotToTurret) {
    this(
        turret,
        drive,
        robotToTurret,
        null,
        OutputMode.AZIMUTH_ONLY,
        () -> 0.0,
        () -> 0.0,
        () -> false);
  }

  public TurretTargeting(
      Turret turret, Drive drive, Translation2d robotToTurret, Translation2d targetOverride) {
    this(
        turret,
        drive,
        robotToTurret,
        () -> targetOverride,
        OutputMode.AZIMUTH_ONLY,
        () -> 0.0,
        () -> 0.0,
        () -> false);
  }

  public TurretTargeting(
      Turret turret,
      Drive drive,
      Translation2d robotToTurret,
      OutputMode outputMode,
      DoubleSupplier hoodAngleDegSupplier,
      DoubleSupplier flywheelRpsSupplier,
      BooleanSupplier runSetpointsSupplier) {
    this(
        turret,
        drive,
        robotToTurret,
        null,
        outputMode,
        hoodAngleDegSupplier,
        flywheelRpsSupplier,
        runSetpointsSupplier);
  }

  public TurretTargeting(
      Turret turret,
      Drive drive,
      Translation2d robotToTurret,
      Supplier<Translation2d> targetOverrideSupplier,
      OutputMode outputMode,
      DoubleSupplier hoodAngleDegSupplier,
      DoubleSupplier flywheelRpsSupplier,
      BooleanSupplier runSetpointsSupplier) {
    this.turret = turret;
    this.drive = drive;
    this.robotToTurret = robotToTurret;
    this.targetOverrideSupplier = targetOverrideSupplier;
    this.outputMode = outputMode;
    this.hoodAngleDegSupplier = hoodAngleDegSupplier;
    this.flywheelRpsSupplier = flywheelRpsSupplier;
    this.runSetpointsSupplier = runSetpointsSupplier;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();
    var fieldVelocity = drive.getFieldRelativeSpeeds();
    Translation2d targetOverride =
        targetOverrideSupplier != null ? targetOverrideSupplier.get() : null;
    var params =
        targetOverride == null
            ? shotCalc.getParameters(robotPose, fieldVelocity, robotToTurret)
            : shotCalc.getParameters(robotPose, fieldVelocity, targetOverride, robotToTurret);

    if (params.isValid()) {
      turret.aimAzimuth(params.turretAngle());
      if (outputMode == OutputMode.MANUAL_SETPOINTS_WHEN_ENABLED) {
        if (shouldRunManualSetpoints(outputMode, runSetpointsSupplier.getAsBoolean())) {
          turret.applyManualSetpoints(
              hoodAngleDegSupplier.getAsDouble(), flywheelRpsSupplier.getAsDouble());
        } else {
          turret.stopFlywheels();
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (outputMode == OutputMode.MANUAL_SETPOINTS_WHEN_ENABLED) {
      turret.stopFlywheels();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static double getDistanceToTargetMeters(
      Pose2d robotPose, Translation2d robotToTurret, Translation2d target) {
    Pose2d turretPose = robotPose.transformBy(new Transform2d(robotToTurret, Rotation2d.kZero));

    return target.getDistance(turretPose.getTranslation());
  }

  public static boolean shouldRunManualSetpoints(OutputMode outputMode, boolean enabled) {
    return outputMode == OutputMode.MANUAL_SETPOINTS_WHEN_ENABLED && enabled;
  }
}
