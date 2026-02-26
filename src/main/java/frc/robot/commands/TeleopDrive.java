package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.TRENCH_ALIGN_TIME_SEC;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SubsystemConstants.Turret;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Zones;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Default teleop drive with trench-safe lock behavior. */
public class TeleopDrive extends Command {
  private static final double DEADBAND = 0.1;

  private static final double TRENCH_Y_KP = 2.0;
  private static final double TRENCH_Y_KI = 0.0;
  private static final double TRENCH_Y_KD = 0.0;
  private static final double TRENCH_Y_TOLERANCE_METERS = 0.05;

  private static final double HEADING_KP = 4.0;
  private static final double HEADING_KI = 0.0;
  private static final double HEADING_KD = 0.15;
  private static final double HEADING_TOLERANCE_RAD = Math.toRadians(3.0);
  private static final double SNAKE_ANGLE_KP = 5.0;
  private static final double SNAKE_ANGLE_KD = 0.4;
  private static final double SNAKE_ANGLE_MAX_VEL = 8.0;
  private static final double SNAKE_ANGLE_MAX_ACCEL = 20.0;

  private final Drive drive;
  private final CommandXboxController controller;
  private final BooleanSupplier trenchDangerSupplier;
  private final BooleanSupplier snakeEnabledSupplier;
  private final BooleanSupplier isIntakingSupplier;
  private final Translation2d turretMidpointRobot;

  private final PIDController trenchYController =
      new PIDController(TRENCH_Y_KP, TRENCH_Y_KI, TRENCH_Y_KD);
  private final PIDController headingController =
      new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);
  private final ProfiledPIDController snakeController =
      new ProfiledPIDController(
          SNAKE_ANGLE_KP,
          0.0,
          SNAKE_ANGLE_KD,
          new TrapezoidProfile.Constraints(SNAKE_ANGLE_MAX_VEL, SNAKE_ANGLE_MAX_ACCEL));

  private DriveMode driveMode = DriveMode.NORMAL;
  private DriveMode lastDriveMode = DriveMode.NORMAL;

  public TeleopDrive(
      Drive drive,
      CommandXboxController controller,
      BooleanSupplier trenchDangerSupplier,
      BooleanSupplier snakeEnabledSupplier,
      BooleanSupplier isIntakingSupplier) {
    this.drive = drive;
    this.controller = controller;
    this.trenchDangerSupplier = trenchDangerSupplier;
    this.snakeEnabledSupplier = snakeEnabledSupplier;
    this.isIntakingSupplier = isIntakingSupplier;
    this.turretMidpointRobot = Turret.robotToTurret1.plus(Turret.robotToTurret2).times(0.5);

    trenchYController.setTolerance(TRENCH_Y_TOLERANCE_METERS);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setTolerance(HEADING_TOLERANCE_RAD);
    snakeController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    driveMode = DriveMode.NORMAL;
    lastDriveMode = DriveMode.NORMAL;
    trenchYController.reset();
    headingController.reset();
    snakeController.reset(drive.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Translation2d joystickLinear =
        getLinearVelocityFromJoysticks(-controller.getLeftY(), -controller.getLeftX());
    boolean isMoving = joystickLinear.getNorm() > 1e-3;
    boolean snakeEnabled = snakeEnabledSupplier.getAsBoolean();
    boolean isIntaking = isIntakingSupplier.getAsBoolean();

    driveMode =
        selectDriveMode(trenchDangerSupplier.getAsBoolean(), snakeEnabled, isIntaking, isMoving);
    maybeResetControllersOnModeChange();

    double xVel = joystickLinear.getX() * drive.getMaxLinearSpeedMetersPerSec();
    double yVel = joystickLinear.getY() * drive.getMaxLinearSpeedMetersPerSec();

    double omegaInput = MathUtil.applyDeadband(-controller.getRightX(), DEADBAND);
    omegaInput = Math.copySign(omegaInput * omegaInput, omegaInput);
    double omegaVel = omegaInput * drive.getMaxAngularSpeedRadPerSec();

    if (driveMode == DriveMode.TRENCH_LOCK) {
      Translation2d turretMidpointField = getTurretMidpointField();
      var activeZone =
          Zones.TRENCH_ZONES.firstWillContain(
              turretMidpointField, drive.getFieldRelativeSpeeds(), TRENCH_ALIGN_TIME_SEC);

      if (activeZone.isPresent()) {
        double ySetpoint = activeZone.get().getYCenter();
        yVel = trenchYController.calculate(drive.getPose().getY(), ySetpoint);
        yVel =
            MathUtil.clamp(
                yVel,
                -drive.getMaxLinearSpeedMetersPerSec(),
                drive.getMaxLinearSpeedMetersPerSec());

        double headingTarget = getFrontBackHeadingTarget(drive.getRotation());
        omegaVel = headingController.calculate(drive.getRotation().getRadians(), headingTarget);
        omegaVel =
            MathUtil.clamp(
                omegaVel,
                -drive.getMaxAngularSpeedRadPerSec(),
                drive.getMaxAngularSpeedRadPerSec());
      } else {
        driveMode = DriveMode.NORMAL;
      }
    } else if (driveMode == DriveMode.SNAKE) {
      Rotation2d targetYaw =
          AllianceFlipUtil.apply(joystickLinear.getAngle().plus(Rotation2d.fromDegrees(180)));
      omegaVel =
          snakeController.calculate(drive.getRotation().getRadians(), targetYaw.getRadians());
      omegaVel =
          MathUtil.clamp(
              omegaVel, -drive.getMaxAngularSpeedRadPerSec(), drive.getMaxAngularSpeedRadPerSec());
    }

    Logger.recordOutput("Drive/TeleopDriveMode", driveMode.toString());
    Logger.recordOutput("Drive/SnakeMode/Enabled", snakeEnabled);
    Logger.recordOutput("Drive/SnakeMode/Active", driveMode == DriveMode.SNAKE);

    ChassisSpeeds fieldSpeeds = new ChassisSpeeds(xVel, yVel, omegaVel);
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldSpeeds, AllianceFlipUtil.apply(drive.getRotation())));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private Translation2d getTurretMidpointField() {
    Pose2d robotPose = drive.getPose();
    return robotPose.getTranslation().plus(turretMidpointRobot.rotateBy(robotPose.getRotation()));
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    linearMagnitude = linearMagnitude * linearMagnitude;
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  private static double getFrontBackHeadingTarget(Rotation2d currentHeading) {
    double currentRad = currentHeading.getRadians();
    double distanceToZero = Math.abs(MathUtil.inputModulus(currentRad, -Math.PI, Math.PI));
    double distanceToPi = Math.abs(MathUtil.inputModulus(currentRad - Math.PI, -Math.PI, Math.PI));
    return distanceToZero <= distanceToPi ? 0.0 : Math.PI;
  }

  static DriveMode selectDriveMode(
      boolean trenchDanger, boolean snakeEnabled, boolean isIntaking, boolean isMoving) {
    if (trenchDanger) {
      return DriveMode.TRENCH_LOCK;
    }
    if (snakeEnabled && isIntaking && isMoving) {
      return DriveMode.SNAKE;
    }
    return DriveMode.NORMAL;
  }

  private void maybeResetControllersOnModeChange() {
    if (driveMode == lastDriveMode) {
      return;
    }

    switch (driveMode) {
      case NORMAL:
        trenchYController.reset();
        headingController.reset();
        break;
      case SNAKE:
        snakeController.reset(drive.getRotation().getRadians());
        break;
      case TRENCH_LOCK:
        trenchYController.reset();
        headingController.reset();
        break;
    }

    lastDriveMode = driveMode;
  }

  enum DriveMode {
    NORMAL,
    SNAKE,
    TRENCH_LOCK
  }
}
