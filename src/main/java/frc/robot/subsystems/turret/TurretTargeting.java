package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/** Default turret command that only keeps azimuth pointed at a field target. */
public class TurretTargeting extends Command {
  private final Turret turret;
  private final Drive drive;
  private final Translation2d robotToTurret;
  private final Translation2d target;
  private final Runnable loopHook;

  public TurretTargeting(
      Turret turret,
      Drive drive,
      Translation2d robotToTurret,
      Translation2d target,
      Runnable loopHook) {
    this.turret = turret;
    this.drive = drive;
    this.robotToTurret = robotToTurret;
    this.target = target;
    this.loopHook = loopHook;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    turret.trackTargetOnly(calculateTargetAngleRobot(drive.getPose(), robotToTurret, target));
    loopHook.run();
  }

  public static Rotation2d calculateTargetAngleRobot(
      Pose2d robotPose, Translation2d robotToTurret, Translation2d target) {
    Pose2d turretPose = robotPose.transformBy(new Transform2d(robotToTurret, Rotation2d.kZero));
    Rotation2d turretAngleField = target.minus(turretPose.getTranslation()).getAngle();
    return turretAngleField.minus(robotPose.getRotation());
  }

  public static double getDistanceToTargetMeters(
      Pose2d robotPose, Translation2d robotToTurret, Translation2d target) {
    Pose2d turretPose = robotPose.transformBy(new Transform2d(robotToTurret, Rotation2d.kZero));
    return target.getDistance(turretPose.getTranslation());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
