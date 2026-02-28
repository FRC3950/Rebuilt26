package frc.robot.subsystems.climber;

import static frc.robot.Constants.SubsystemConstants.CANivore;
import static frc.robot.Constants.SubsystemConstants.Climber.*;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Climber extends SubsystemBase {
  private enum ClimbSide {
    NORTH,
    SOUTH
  }

  private final TalonFX climberMotor;
  private final DigitalInput limitSwitch;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);
  private final PathConstraints climbPathConstraints =
      new PathConstraints(
          climbPathMaxVelocityMetersPerSec,
          climbPathMaxAccelerationMetersPerSecSq,
          Units.degreesToRadians(climbPathMaxAngularVelocityDegPerSec),
          Units.degreesToRadians(climbPathMaxAngularAccelerationDegPerSecSq));

  private boolean inClimb = false;
  private ClimbSide lastClimbSide = ClimbSide.NORTH;

  public Climber(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    climberMotor = new TalonFX(climberMotorID, CANivore);
    limitSwitch = new DigitalInput(limitSwitchPort);
    climberMotor.getConfigurator().apply(climberConfig);

    // Reset position to 0 when limit switch is triggered
    new Trigger(limitSwitch::get).onTrue(runOnce(() -> climberMotor.setPosition(0)));
  }

  public void setPosition(double targetPosition) {
    climberMotor.setControl(mmRequest.withPosition(targetPosition));
  }

  @AutoLogOutput(key = "Climber/Climber Zero'd?")
  public boolean getClimberZeroed() {
    return limitSwitch.get();
  }

  @AutoLogOutput(key = "Climber/Climber Position")
  public double getClimberPosition() {
    return climberMotor.getPosition().getValueAsDouble();
  }

  private ClimbSide getClosestSide() {
    Translation2d robotTranslation = robotPoseSupplier.get().getTranslation();
    double northDistance = robotTranslation.getDistance(Constants.FieldConstants.climbNorth);
    double southDistance = robotTranslation.getDistance(Constants.FieldConstants.climbSouth);
    return northDistance <= southDistance ? ClimbSide.NORTH : ClimbSide.SOUTH;
  }

  private Translation2d getClimbPoint(ClimbSide side) {
    return side == ClimbSide.NORTH
        ? Constants.FieldConstants.climbNorth
        : Constants.FieldConstants.climbSouth;
  }

  private Pose2d toClimbPose(ClimbSide side) {
    return new Pose2d(getClimbPoint(side), Rotation2d.fromDegrees(climbHeadingDeg));
  }

  private Pose2d toApproachPose(ClimbSide side) {
    return new Pose2d(
        getOffsetPoint(side, climbOffsetMeters), Rotation2d.fromDegrees(climbHeadingDeg));
  }

  private Pose2d toBackoffPose(ClimbSide side) {
    return new Pose2d(
        getOffsetPoint(side, climbOffsetMeters), Rotation2d.fromDegrees(climbHeadingDeg));
  }

  private Translation2d getOffsetPoint(ClimbSide side, double offsetMeters) {
    double direction = side == ClimbSide.NORTH ? 1.0 : -1.0;
    return getClimbPoint(side).plus(new Translation2d(0.0, offsetMeters * direction));
  }

  private Command pathfindTo(Pose2d targetPose) {
    return AutoBuilder.pathfindToPose(targetPose, climbPathConstraints);
  }

  private Command moveClimberTo(double targetPosition) {
    return runOnce(() -> setPosition(targetPosition))
        .andThen(
            Commands.waitUntil(
                    () ->
                        Math.abs(getClimberPosition() - targetPosition) <= climberPositionTolerance)
                .withTimeout(climberMoveTimeoutSecs));
  }

  private Command createClimbSequence() {
    return Commands.sequence(
            moveClimberTo(climbUpHeight),
            runOnce(() -> lastClimbSide = getClosestSide()),
            Commands.defer(() -> pathfindTo(toApproachPose(lastClimbSide)), Set.of()),
            Commands.defer(() -> pathfindTo(toClimbPose(lastClimbSide)), Set.of()),
            moveClimberTo(climbFinalPos))
        .andThen(runOnce(() -> inClimb = true));
  }

  private Command createUnclimbSequence() {
    return Commands.sequence(
            moveClimberTo(climbUpHeight),
            Commands.defer(() -> pathfindTo(toBackoffPose(lastClimbSide)), Set.of()),
            moveClimberTo(climbFinalPos))
        .andThen(runOnce(() -> inClimb = false));
  }

  public Command ClimbToggleCommand() {
    return Commands.defer(
        () -> inClimb ? createUnclimbSequence() : createClimbSequence(), Set.of(this));
  }
}
