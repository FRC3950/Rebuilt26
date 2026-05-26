package frc.robot.subsystems.turret;

import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret1;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret2;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class TurretVisualization extends SubsystemBase {
  private static final int REAL_LOG_PERIOD_LOOPS = 5;
  private static final Pose3d[] ZEROED_COMPONENT_POSES = {new Pose3d(), new Pose3d()};

  private final Turret turret1;
  private final Turret turret2;
  private final Pose3d[] finalComponentPoses = new Pose3d[2];
  private int loopCounter = 0;

  public TurretVisualization(Turret turret1, Turret turret2) {
    this.turret1 = turret1;
    this.turret2 = turret2;
  }

  @Override
  public void periodic() {
    boolean replay = Constants.currentMode == Constants.Mode.REPLAY;
    boolean shouldLog = replay || loopCounter % REAL_LOG_PERIOD_LOOPS == 0;
    loopCounter++;

    if (!shouldLog) {
      return;
    }

    finalComponentPoses[0] = turret1.getRobotPose3d(robotToTurret1);
    finalComponentPoses[1] = turret2.getRobotPose3d(robotToTurret2);

    Logger.recordOutput("ZeroedComponentPoses", ZEROED_COMPONENT_POSES);
    Logger.recordOutput("FinalComponentPoses", finalComponentPoses);
  }
}
