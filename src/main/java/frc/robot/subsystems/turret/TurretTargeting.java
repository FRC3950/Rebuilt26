package frc.robot.subsystems.turret;

import static frc.robot.Constants.FieldConstants.leftFerryTarget;
import static frc.robot.Constants.FieldConstants.rightFerryTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

/**
 * Default turret targeting command.
 *
 * <p>Calculates compensated shot parameters (shoot-on-the-move) and feeds the turret/hood/flywheel
 * controllers each loop.
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
  private final GetAdjustedShot shotCalc = GetAdjustedShot.getInstance();

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

  public static TargetingMode selectTargetingMode(boolean trenchDanger, boolean inNeutralZone) {
    if (trenchDanger) {
      return TargetingMode.HUB_AUTO;
    }
    if (inNeutralZone) {
      return TargetingMode.FERRY_AUTO;
    }
    return TargetingMode.HUB_AUTO;
  }

  @Override
  public void execute() {
    // Match MA style: clear cache once per loop, then calculate.
    shotCalc.clearShootingParameters();
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds fieldSpeeds = drive.getFieldRelativeSpeeds();
    var params =
        switch (modeSupplier.get()) {
          case FERRY_AUTO -> {
            double distLeft = robotPose.getTranslation().getDistance(leftFerryTarget);
            double distRight = robotPose.getTranslation().getDistance(rightFerryTarget);
            Translation2d target = (distLeft < distRight) ? leftFerryTarget : rightFerryTarget;
            yield shotCalc.getParameters(robotPose, fieldSpeeds, target, robotToTurret);
          }
          case HUB_AUTO -> shotCalc.getParameters(robotPose, fieldSpeeds, robotToTurret);
        };

    if (params.isValid()) {
      turret.runAutoTarget(params);
    } else {
      System.out.println("Invalid shot parameters");
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
