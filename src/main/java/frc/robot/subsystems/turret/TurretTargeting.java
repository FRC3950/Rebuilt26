package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/**
 * Default turret targeting command.
 *
 * <p>Calculates compensated shot parameters (shoot-on-the-move) and feeds the turret/hood/flywheel
 * controllers each loop.
 */
public class TurretTargeting extends Command {
  private final Turret turret;
  private final Drive drive;
  private final Translation2d robotToTurret;
  private final GetAdjustedShot shotCalc = GetAdjustedShot.getInstance();

  public TurretTargeting(Turret turret, Drive drive, Translation2d robotToTurret) {
    this.turret = turret;
    this.drive = drive;
    this.robotToTurret = robotToTurret;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    // Match MA style: clear cache once per loop, then calculate.
    shotCalc.clearShootingParameters();
    var params =
        shotCalc.getParameters(drive.getPose(), drive.getFieldRelativeSpeeds(), robotToTurret);

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
