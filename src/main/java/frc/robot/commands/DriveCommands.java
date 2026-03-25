// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.hubTranslation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double LINEAR_ACCEL_LIMIT_METERS_PER_SEC_SQ = 3.0;
  private static final double ANGULAR_ACCEL_LIMIT_RAD_PER_SEC_SQ = 8.0;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double ALIGN_Y_KP = 2.0;
  private static final double ALIGN_Y_KD = 0.0;
  private static final double ALIGN_Y_MAX_VELOCITY = 2.0;
  private static final double ALIGN_Y_MAX_ACCELERATION = 4.0;
  private static final double DISTANCE_KP = 2.0;
  private static final double DISTANCE_KD = 0.0;
  private static final double DISTANCE_MAX_VELOCITY = 2.0;
  private static final double DISTANCE_MAX_ACCELERATION = 4.0;
  private static final double DISTANCE_TOLERANCE_METERS = 0.05;
  private static final double DPAD_NUDGE_SPEED_METERS_PER_SECOND = 0.2;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  private static double limitAccelerationOnly(
      double currentValue, double targetValue, double maxDelta) {
    if (Math.abs(targetValue) <= Math.abs(currentValue)) {
      return targetValue;
    }

    if (Math.signum(currentValue) != 0.0 && Math.signum(currentValue) != Math.signum(targetValue)) {
      return 0.0;
    }

    return currentValue + MathUtil.clamp(targetValue - currentValue, -maxDelta, maxDelta);
  }

  private static class AccelOnlyLimiter {
    private Translation2d currentLinearVelocity = Translation2d.kZero;
    private double currentOmega = 0.0;

    public ChassisSpeeds calculate(
        Translation2d targetLinearVelocity,
        double targetOmega,
        double maxLinearDelta,
        double maxOmegaDelta) {
      double currentMagnitude = currentLinearVelocity.getNorm();
      double targetMagnitude = targetLinearVelocity.getNorm();

      double limitedMagnitude =
          limitAccelerationOnly(currentMagnitude, targetMagnitude, maxLinearDelta);
      Translation2d limitedLinearVelocity =
          targetMagnitude > 1e-6
              ? new Translation2d(limitedMagnitude, targetLinearVelocity.getAngle())
              : Translation2d.kZero;

      currentLinearVelocity = limitedLinearVelocity;
      currentOmega = limitAccelerationOnly(currentOmega, targetOmega, maxOmegaDelta);

      return new ChassisSpeeds(
          currentLinearVelocity.getX(), currentLinearVelocity.getY(), currentOmega);
    }

    public void reset() {
      currentLinearVelocity = Translation2d.kZero;
      currentOmega = 0.0;
    }
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    AccelOnlyLimiter limiter = new AccelOnlyLimiter();

    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Apply rotation deadband
              double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

              // Square rotation value for more precise control
              omega = Math.copySign(omega * omega, omega);

              ChassisSpeeds speeds =
                  limiter.calculate(
                      linearVelocity.times(drive.getMaxLinearSpeedMetersPerSec()),
                      omega * drive.getMaxAngularSpeedRadPerSec(),
                      LINEAR_ACCEL_LIMIT_METERS_PER_SEC_SQ * 0.02,
                      ANGULAR_ACCEL_LIMIT_RAD_PER_SEC_SQ * 0.02);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds, AllianceFlipUtil.apply(drive.getRotation())));
            },
            drive)
        .beforeStarting(limiter::reset);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds, AllianceFlipUtil.apply(drive.getRotation())));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /** Aligns the robot so its Y position matches the hub and the back of the robot faces the hub. */
  public static Command alignRobotToHubLine(Drive drive, Translation2d robotToTurret) {
    ProfiledPIDController yController =
        new ProfiledPIDController(
            ALIGN_Y_KP,
            0.0,
            ALIGN_Y_KD,
            new TrapezoidProfile.Constraints(ALIGN_Y_MAX_VELOCITY, ALIGN_Y_MAX_ACCELERATION));
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              Pose2d currentPose = drive.getPose();
              Pose2d targetPose = getHubLineTargetPose(currentPose, robotToTurret);

              double yVelocity =
                  yController.calculate(
                      currentPose.getY(), targetPose.getTranslation().getY());
              double omega =
                  angleController.calculate(
                      currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

              ChassisSpeeds speeds = new ChassisSpeeds(0.0, yVelocity, omega);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              Pose2d currentPose = drive.getPose();
              Pose2d targetPose = getHubLineTargetPose(currentPose, robotToTurret);
              yController.reset(currentPose.getY());
              angleController.reset(currentPose.getRotation().getRadians());
              yController.setGoal(targetPose.getY());
              angleController.setGoal(targetPose.getRotation().getRadians());
            });
  }

  /** Drives the robot until turret 1 reaches the requested distance from the hub. */
  public static Command driveLeftTurretToHubDistance(
      Drive drive, Translation2d robotToTurret, double targetDistanceMeters) {
    ProfiledPIDController distanceController =
        new ProfiledPIDController(
            DISTANCE_KP,
            0.0,
            DISTANCE_KD,
            new TrapezoidProfile.Constraints(
                DISTANCE_MAX_VELOCITY, DISTANCE_MAX_ACCELERATION));

    return Commands.run(
            () -> {
              Pose2d currentPose = drive.getPose();
              double currentDistance = getTurretDistanceToHubMeters(currentPose, robotToTurret);
              Translation2d translationDirection =
                  getHubDistanceTranslationDirection(currentPose, robotToTurret);
              double speedMetersPerSecond =
                  distanceController.calculate(currentDistance, targetDistanceMeters);

              Translation2d fieldVelocity = translationDirection.times(speedMetersPerSecond);
              ChassisSpeeds speeds =
                  new ChassisSpeeds(fieldVelocity.getX(), fieldVelocity.getY(), 0.0);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              double currentDistance = getTurretDistanceToHubMeters(drive.getPose(), robotToTurret);
              distanceController.reset(currentDistance);
              distanceController.setGoal(targetDistanceMeters);
              distanceController.setTolerance(DISTANCE_TOLERANCE_METERS);
            })
        .until(
            () ->
                isWithinDistanceTolerance(
                    getTurretDistanceToHubMeters(drive.getPose(), robotToTurret),
                    targetDistanceMeters))
        .finallyDo(drive::stop);
  }

  /** Drives the robot at a small constant robot-relative speed while held. */
  public static Command robotRelativeNudge(
      Drive drive, double vxMetersPerSecond, double vyMetersPerSecond) {
    return Commands.run(
            () -> drive.runVelocity(new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, 0.0)),
            drive)
        .finallyDo(drive::stop);
  }

  public static double getDpadNudgeSpeedMetersPerSecond() {
    return DPAD_NUDGE_SPEED_METERS_PER_SECOND;
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }

  public static Pose2d getHubLineTargetPose(Pose2d currentPose, Translation2d robotToTurret) {
    Translation2d provisionalTranslation = new Translation2d(currentPose.getX(), hubTranslation.getY());
    Rotation2d targetHeading = getBackFacingHeading(provisionalTranslation, currentPose);
    double targetRobotY = hubTranslation.getY() - robotToTurret.rotateBy(targetHeading).getY();
    Translation2d targetTranslation = new Translation2d(currentPose.getX(), targetRobotY);
    return new Pose2d(targetTranslation, targetHeading);
  }

  public static Rotation2d getBackFacingHeading(
      Translation2d robotTranslation, Pose2d fallbackPoseForHeading) {
    Translation2d robotToHub = hubTranslation.minus(robotTranslation);
    if (robotToHub.getNorm() < 1e-6) {
      return fallbackPoseForHeading.getRotation();
    }
    return robotToHub.getAngle().rotateBy(new Rotation2d(Math.PI));
  }

  public static double getTurretDistanceToHubMeters(
      Pose2d robotPose, Translation2d robotToTurret) {
    return robotPose
        .transformBy(new Transform2d(robotToTurret, Rotation2d.kZero))
        .getTranslation()
        .getDistance(hubTranslation);
  }

  public static Translation2d getHubDistanceTranslationDirection(
      Pose2d robotPose, Translation2d robotToTurret) {
    Translation2d turretTranslation =
        robotPose.transformBy(new Transform2d(robotToTurret, Rotation2d.kZero)).getTranslation();
    Translation2d hubToTurret = turretTranslation.minus(hubTranslation);
    if (hubToTurret.getNorm() < 1e-6) {
      return new Translation2d(1.0, 0.0);
    }
    return hubToTurret.div(hubToTurret.getNorm());
  }

  public static Pose2d getLeftTurretHubDistanceTargetPose(
      Pose2d robotPose, Translation2d robotToTurret, double targetDistanceMeters) {
    Translation2d direction = getHubDistanceTranslationDirection(robotPose, robotToTurret);
    Translation2d targetTurretTranslation =
        hubTranslation.plus(direction.times(targetDistanceMeters));
    Translation2d robotToTurretField = robotToTurret.rotateBy(robotPose.getRotation());
    Translation2d targetRobotTranslation = targetTurretTranslation.minus(robotToTurretField);
    return new Pose2d(targetRobotTranslation, robotPose.getRotation());
  }

  public static boolean isWithinDistanceTolerance(
      double currentDistanceMeters, double targetDistanceMeters) {
    return Math.abs(currentDistanceMeters - targetDistanceMeters) <= DISTANCE_TOLERANCE_METERS;
  }
}
