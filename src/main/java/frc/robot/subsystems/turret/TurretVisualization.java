package frc.robot.subsystems.turret;

import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret1;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret2;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class TurretVisualization extends SubsystemBase {
  private final Turret turret1;
  private final Turret turret2;

  public TurretVisualization(Turret turret1, Turret turret2) {
    this.turret1 = turret1;
    this.turret2 = turret2;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("ZeroedComponentPoses", new Pose3d[] {new Pose3d(), new Pose3d()});
    Logger.recordOutput(
        "FinalComponentPoses",
        new Pose3d[] {
          turret1.getRobotPose3d(robotToTurret1), turret2.getRobotPose3d(robotToTurret2)
        });
  }
}
