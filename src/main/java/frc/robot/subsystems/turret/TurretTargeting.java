package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/**
 * Default turret targeting command.
 *
 * <p>Calculates turret setpoints from the robot's current pose and feeds the
 * turret/hood/flywheel controllers each loop.
 */
public class TurretTargeting extends Command {
  private final Turret turret;
  private final Drive drive;
  private final Translation2d robotToTurret;
  private final Translation2d targetOverride;
  private final GetAdjustedShot shotCalc = new GetAdjustedShot();

  public TurretTargeting(Turret turret, Drive drive, Translation2d robotToTurret) {
    this(turret, drive, robotToTurret, null);
  }

  public TurretTargeting(
      Turret turret, Drive drive, Translation2d robotToTurret, Translation2d targetOverride) {
    this.turret = turret;
    this.drive = drive;
    this.robotToTurret = robotToTurret;
    this.targetOverride = targetOverride;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();
    var params =
        targetOverride == null
            ? shotCalc.getParameters(robotPose, robotToTurret)
            : shotCalc.getParameters(robotPose, targetOverride, robotToTurret);

    if (params.isValid()) {
      turret.runAutoTarget(params);
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
