package frc.robot.sim;

import static frc.robot.Constants.SimConstants.Fuel.EFFECTIVE_FLYWHEEL_RADIUS_METERS;
import static frc.robot.Constants.SimConstants.Fuel.EXIT_VELOCITY_SCALE;
import static frc.robot.Constants.SimConstants.Fuel.INTAKE_CENTER;
import static frc.robot.Constants.SimConstants.Fuel.TURRET_LAUNCH_HEIGHT_METERS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Distancer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class FuelLaunchCalculator {
  public record LaunchState(Translation3d position, Translation3d velocity) {}

  static class LaunchCalibration {
    private final List<CalibrationPoint> hoodAnglePoints;
    private final List<CalibrationPoint> flywheelSpeedPoints;

    private LaunchCalibration(
        List<CalibrationPoint> hoodAnglePoints, List<CalibrationPoint> flywheelSpeedPoints) {
      this.hoodAnglePoints = hoodAnglePoints;
      this.flywheelSpeedPoints = flywheelSpeedPoints;
    }

    static LaunchCalibration fromShotTable() {
      List<Distancer.Row> rows = Distancer.loadRowsFromDeploy("shot_table.json");
      if (rows.isEmpty()) {
        return identity();
      }

      Map<Double, List<Double>> hoodAngleSamples = new HashMap<>();
      Map<Double, List<Double>> launchSpeedSamples = new HashMap<>();

      for (Distancer.Row row : rows) {
        if (row.tof <= 0.0 || row.rps <= 0.0) {
          continue;
        }

        double horizontalSpeedMetersPerSecond = row.d / row.tof;
        double verticalSpeedMetersPerSecond =
            (FuelSim.Hub.ENTRY_HEIGHT - TURRET_LAUNCH_HEIGHT_METERS
                    + 0.5 * 9.81 * row.tof * row.tof)
                / row.tof;
        double launchAngleDeg =
            Units.radiansToDegrees(
                Math.atan2(verticalSpeedMetersPerSecond, horizontalSpeedMetersPerSecond));
        double launchSpeedMetersPerSecond =
            Math.hypot(horizontalSpeedMetersPerSecond, verticalSpeedMetersPerSecond);

        hoodAngleSamples
            .computeIfAbsent(row.hoodDeg, ignored -> new ArrayList<>())
            .add(launchAngleDeg);
        launchSpeedSamples
            .computeIfAbsent(row.rps, ignored -> new ArrayList<>())
            .add(launchSpeedMetersPerSecond);
      }

      if (hoodAngleSamples.size() < 2 || launchSpeedSamples.size() < 2) {
        return identity();
      }

      return new LaunchCalibration(
          averageSamples(hoodAngleSamples), averageSamples(launchSpeedSamples));
    }

    static LaunchCalibration identity() {
      return new LaunchCalibration(
          List.of(
              new CalibrationPoint(0.0, 0.0),
              new CalibrationPoint(90.0, 90.0)),
          List.of(
              new CalibrationPoint(0.0, 0.0),
              new CalibrationPoint(
                  100.0,
                  2.0 * Math.PI * EFFECTIVE_FLYWHEEL_RADIUS_METERS * 100.0)));
    }

    double launchAngleForHoodDeg(double hoodAngleDeg) {
      return interpolate(hoodAnglePoints, hoodAngleDeg);
    }

    double launchSpeedForFlywheelRps(double flywheelRps) {
      return interpolate(flywheelSpeedPoints, flywheelRps) * EXIT_VELOCITY_SCALE;
    }

    private static List<CalibrationPoint> averageSamples(Map<Double, List<Double>> samples) {
      List<CalibrationPoint> points = new ArrayList<>();
      for (var entry : samples.entrySet()) {
        double average =
            entry.getValue().stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
        points.add(new CalibrationPoint(entry.getKey(), average));
      }
      points.sort(Comparator.comparingDouble(CalibrationPoint::input));
      return points;
    }

    private static double interpolate(List<CalibrationPoint> points, double input) {
      if (points.size() == 1) {
        return points.get(0).output();
      }

      CalibrationPoint lower = points.get(0);
      CalibrationPoint upper = points.get(points.size() - 1);

      if (input <= lower.input()) {
        upper = points.get(1);
      } else if (input >= upper.input()) {
        lower = points.get(points.size() - 2);
      } else {
        for (int i = 0; i < points.size() - 1; i++) {
          CalibrationPoint candidateLower = points.get(i);
          CalibrationPoint candidateUpper = points.get(i + 1);
          if (input >= candidateLower.input() && input <= candidateUpper.input()) {
            lower = candidateLower;
            upper = candidateUpper;
            break;
          }
        }
      }

      double inputSpan = upper.input() - lower.input();
      if (Math.abs(inputSpan) < 1e-9) {
        return lower.output();
      }

      double t = (input - lower.input()) / inputSpan;
      return lower.output() + (upper.output() - lower.output()) * t;
    }

    private record CalibrationPoint(double input, double output) {}
  }

  private final LaunchCalibration calibration;

  public FuelLaunchCalculator() {
    this(LaunchCalibration.fromShotTable());
  }

  FuelLaunchCalculator(LaunchCalibration calibration) {
    this.calibration = calibration;
  }

  public LaunchState calculateTurretLaunch(
      Pose2d robotPose,
      ChassisSpeeds fieldSpeeds,
      Translation2d robotToTurret,
      double turretYawDeg,
      double hoodAngleDeg,
      double flywheelRps) {
    Translation2d fieldOffset = robotToTurret.rotateBy(robotPose.getRotation());
    Translation2d fieldPosition = robotPose.getTranslation().plus(fieldOffset);
    double hoodRadians = Units.degreesToRadians(calibration.launchAngleForHoodDeg(hoodAngleDeg));
    double fieldYawRadians =
        robotPose.getRotation().getRadians() + Units.degreesToRadians(turretYawDeg);
    double launchSpeedMetersPerSecond = calibration.launchSpeedForFlywheelRps(flywheelRps);
    double horizontalSpeedMetersPerSecond = Math.cos(hoodRadians) * launchSpeedMetersPerSecond;

    return new LaunchState(
        new Translation3d(fieldPosition.getX(), fieldPosition.getY(), TURRET_LAUNCH_HEIGHT_METERS),
        new Translation3d(
            horizontalSpeedMetersPerSecond * Math.cos(fieldYawRadians)
                + fieldSpeeds.vxMetersPerSecond,
            horizontalSpeedMetersPerSecond * Math.sin(fieldYawRadians)
                + fieldSpeeds.vyMetersPerSecond,
            Math.sin(hoodRadians) * launchSpeedMetersPerSecond));
  }

  public LaunchState calculateOuttakeLaunch(
      Pose2d robotPose,
      ChassisSpeeds fieldSpeeds,
      double releaseHeightMeters,
      double outtakeSpeedMetersPerSecond,
      double lateralOffsetMeters) {
    Translation2d fieldOffset =
        INTAKE_CENTER.plus(new Translation2d(0.0, lateralOffsetMeters)).rotateBy(robotPose.getRotation());
    Translation2d fieldPosition = robotPose.getTranslation().plus(fieldOffset);
    double fieldYawRadians = robotPose.getRotation().getRadians();

    return new LaunchState(
        new Translation3d(fieldPosition.getX(), fieldPosition.getY(), releaseHeightMeters),
        new Translation3d(
            outtakeSpeedMetersPerSecond * Math.cos(fieldYawRadians) + fieldSpeeds.vxMetersPerSecond,
            outtakeSpeedMetersPerSecond * Math.sin(fieldYawRadians) + fieldSpeeds.vyMetersPerSecond,
            0.0));
  }
}
