package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Objects;
import java.util.function.Supplier;

/** Publishes the robot pose to a SmartDashboard Field2d widget. */
public class Field2dPublisher extends SubsystemBase {
  private final Supplier<Pose2d> poseSupplier;
  private final Field2d field = new Field2d();

  public Field2dPublisher(String dashboardKey, Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = Objects.requireNonNull(poseSupplier);

    SmartDashboard.putData(Objects.requireNonNull(dashboardKey), field);
    field.setRobotPose(this.poseSupplier.get());
  }

  public Field2d getField() {
    return field;
  }

  @Override
  public void periodic() {
    field.setRobotPose(poseSupplier.get());
  }
}
