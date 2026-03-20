package frc.robot.subsystems.turret;

import static frc.robot.Constants.FieldConstants.leftFerryTarget;
import static frc.robot.Constants.FieldConstants.rightFerryTarget;

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
 * <p>Calculates turret setpoints from the robot's current pose and feeds the
 * turret/hood/flywheel controllers each loop.
 */
public class TurretTargeting extends Command {
  public enum TargetingMode {
    HUB_AUTO,
    FERRY_AUTO
  }

  private final Turret turret;
  private final Drive drive;
  private final Translation2d robotToTurret;
  private final Supplier<TargetingMode> modeSupplier;
  private final GetAdjustedShot shotCalc = new GetAdjustedShot();

  public TurretTargeting(
      Turret turret,
      Drive drive,
      Translation2d robotToTurret,
      Supplier<TargetingMode> modeSupplier) {
    this.turret = turret;
    this.drive = drive;
    this.robotToTurret = robotToTurret;
    this.modeSupplier = modeSupplier;
    addRequirements(turret);
  }

  public static TargetingMode selectTargetingMode(
      boolean trenchSafetyEnabled, boolean trenchDanger, boolean inNeutralZone) {
    if (trenchSafetyEnabled && trenchDanger) {
      return TargetingMode.HUB_AUTO;
    }
    if (inNeutralZone) {
      return TargetingMode.FERRY_AUTO;
    }
    return TargetingMode.HUB_AUTO;
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();
    var params =
        switch (modeSupplier.get()) {
          case FERRY_AUTO -> {
            double distLeft = robotPose.getTranslation().getDistance(leftFerryTarget);
            double distRight = robotPose.getTranslation().getDistance(rightFerryTarget);
            Translation2d target = (distLeft < distRight) ? leftFerryTarget : rightFerryTarget;
            yield shotCalc.getParameters(robotPose, target, robotToTurret);
          }
          case HUB_AUTO -> shotCalc.getParameters(robotPose, robotToTurret);
        };

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
