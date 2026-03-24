package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

/**
 * Default turret targeting command.
 *
 * <p>Calculates turret setpoints from the robot's current pose and feeds the turret/hood/flywheel
 * controllers each loop.
 */
public class TurretTargeting extends Command {
  private final Turret turret;
  private final Drive drive;
  private final Translation2d robotToTurret;
  private final Supplier<Translation2d> targetOverrideSupplier;
  private final boolean lockAzimuthToZero;
  private final GetAdjustedShot shotCalc = new GetAdjustedShot();

  public TurretTargeting(Turret turret, Drive drive, Translation2d robotToTurret) {
    this(turret, drive, robotToTurret, null, false);
  }

  public TurretTargeting(
      Turret turret, Drive drive, Translation2d robotToTurret, boolean lockAzimuthToZero) {
    this(turret, drive, robotToTurret, null, lockAzimuthToZero);
  }

  public TurretTargeting(
      Turret turret, Drive drive, Translation2d robotToTurret, Translation2d targetOverride) {
    this(turret, drive, robotToTurret, () -> targetOverride, false);
  }

  public TurretTargeting(
      Turret turret,
      Drive drive,
      Translation2d robotToTurret,
      Supplier<Translation2d> targetOverrideSupplier) {
    this(turret, drive, robotToTurret, targetOverrideSupplier, false);
  }

  private TurretTargeting(
      Turret turret,
      Drive drive,
      Translation2d robotToTurret,
      Supplier<Translation2d> targetOverrideSupplier,
      boolean lockAzimuthToZero) {
    this.turret = turret;
    this.drive = drive;
    this.robotToTurret = robotToTurret;
    this.targetOverrideSupplier = targetOverrideSupplier;
    this.lockAzimuthToZero = lockAzimuthToZero;
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
      if (Turret.getTargetingMode()) {
        turret.runZeroAzimuthTarget(params);
      } else {
        turret.runAutoTarget(params);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  public static double getDistanceToTargetMeters(
      Pose2d robotPose, Translation2d robotToTurret, Translation2d target) {
    Pose2d turretPose = robotPose.transformBy(new Transform2d(robotToTurret, Rotation2d.kZero));

    return target.getDistance(turretPose.getTranslation());
  }
}
