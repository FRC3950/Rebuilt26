package frc.robot.subsystems.turret;

import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class FerryMode extends Command {
  private final Turret turret;
  private final Drive drive;
  private final GetAdjustedShot calculator;

  private final Translation2d robotToTurret;
  private final Translation2d forcedTarget;

  public FerryMode(Turret turret, Drive drive, Translation2d robotToTurret) {
    this(turret, drive, robotToTurret, null);
  }

  public FerryMode(
      Turret turret, Drive drive, Translation2d robotToTurret, Translation2d forcedTarget) {
    this.turret = turret;
    this.drive = drive;
    this.robotToTurret = robotToTurret;
    this.calculator = GetAdjustedShot.getInstance();
    this.forcedTarget = forcedTarget;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();

    Translation2d target;
    if (forcedTarget != null) {
      target = forcedTarget;
    } else {
      double distLeft = robotPose.getTranslation().getDistance(leftFerryTarget);
      double distRight = robotPose.getTranslation().getDistance(rightFerryTarget);
      target = (distLeft < distRight) ? leftFerryTarget : rightFerryTarget;
    }

    ChassisSpeeds fieldVelocity = drive.getFieldRelativeSpeeds();
    var params = calculator.getParameters(robotPose, fieldVelocity, target, robotToTurret);

    if (params.isValid()) {
      turret.runAutoTarget(params);
      return;
    }
  }
}
