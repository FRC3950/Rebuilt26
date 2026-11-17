package frc.robot.subsystems.turret.demo;

import frc.robot.util.Distancer;
import java.util.List;

/**
 * Drag-free projectile model for solving a shot at any (distance, height).
 *
 * <p>The hood and flywheel are mapped to a launch angle and exit speed by fitting two straight
 * lines to the hub shot table. Each table row's distance and time of flight, plus the known hub
 * impact point (back of the funnel), give the angle and speed the ball actually launched at. The
 * fit therefore follows whatever the hub table is tuned to, and drag at hub ranges is folded in.
 */
public final class BallisticShotModel {
  public static final double GRAVITY = 9.81;

  private final boolean calibrated;
  // launchAngleDeg = angleInterceptDeg + angleSlope * hoodDeg
  private final double angleInterceptDeg;
  private final double angleSlope;
  // exitSpeedMps = speedIntercept + speedSlope * rps
  private final double speedIntercept;
  private final double speedSlope;

  public record Solution(
      boolean valid, double hoodDeg, double flywheelRps, double tofSec, String reason) {
    static Solution invalid(String reason) {
      return new Solution(false, 0.0, 0.0, 0.0, reason);
    }
  }

  BallisticShotModel(
      double angleInterceptDeg, double angleSlope, double speedIntercept, double speedSlope) {
    this.angleInterceptDeg = angleInterceptDeg;
    this.angleSlope = angleSlope;
    this.speedIntercept = speedIntercept;
    this.speedSlope = speedSlope;
    calibrated =
        Double.isFinite(angleInterceptDeg)
            && Double.isFinite(angleSlope)
            && Double.isFinite(speedIntercept)
            && Double.isFinite(speedSlope)
            && speedSlope > 0.0;
  }

  /**
   * Fits the model to hub shot table rows.
   *
   * @param launchHeightMeters ball exit height
   * @param impactHeightMeters height the table's shots land at (back of the funnel)
   * @param impactBeyondDistanceMeters how far past the table's measured distance the ball lands
   */
  public static BallisticShotModel fitFromShotTable(
      List<Distancer.Row> rows,
      double launchHeightMeters,
      double impactHeightMeters,
      double impactBeyondDistanceMeters) {
    double dz = impactHeightMeters - launchHeightMeters;
    int n = 0;
    double[] hood = new double[rows.size()];
    double[] angle = new double[rows.size()];
    double[] rps = new double[rows.size()];
    double[] speed = new double[rows.size()];
    for (Distancer.Row row : rows) {
      if (row.tof <= 0.0 || row.rps <= 0.0) {
        continue;
      }
      double horizontal = row.d + impactBeyondDistanceMeters;
      double vx = horizontal / row.tof;
      double vz = (dz + 0.5 * GRAVITY * row.tof * row.tof) / row.tof;
      hood[n] = row.hoodDeg;
      angle[n] = Math.toDegrees(Math.atan2(vz, vx));
      rps[n] = row.rps;
      speed[n] = Math.hypot(vx, vz);
      n++;
    }
    double[] angleFit = fitLine(hood, angle, n);
    double[] speedFit = fitLine(rps, speed, n);
    return new BallisticShotModel(angleFit[0], angleFit[1], speedFit[0], speedFit[1]);
  }

  /** Least-squares line {intercept, slope}; NaN if fewer than 2 distinct x values. */
  private static double[] fitLine(double[] x, double[] y, int n) {
    if (n < 2) {
      return new double[] {Double.NaN, Double.NaN};
    }
    double meanX = 0.0;
    double meanY = 0.0;
    for (int i = 0; i < n; i++) {
      meanX += x[i];
      meanY += y[i];
    }
    meanX /= n;
    meanY /= n;
    double sxx = 0.0;
    double sxy = 0.0;
    for (int i = 0; i < n; i++) {
      sxx += (x[i] - meanX) * (x[i] - meanX);
      sxy += (x[i] - meanX) * (y[i] - meanY);
    }
    if (sxx < 1e-9) {
      return new double[] {Double.NaN, Double.NaN};
    }
    double slope = sxy / sxx;
    return new double[] {meanY - slope * meanX, slope};
  }

  public boolean isCalibrated() {
    return calibrated;
  }

  public double launchAngleDegForHood(double hoodDeg) {
    return angleInterceptDeg + angleSlope * hoodDeg;
  }

  public double exitSpeedForRps(double rps) {
    return speedIntercept + speedSlope * rps;
  }

  public double rpsForExitSpeed(double speedMps) {
    return (speedMps - speedIntercept) / speedSlope;
  }

  /**
   * Finds the hood angle and flywheel speed that put the ball through a point.
   *
   * @param horizontalDistanceMeters turret to aim point, horizontal
   * @param heightAboveLaunchMeters aim point height minus launch height (may be negative)
   * @param hoodOverrideDeg if within [minHoodDeg, maxHoodDeg], use only this hood angle
   * @param preferredHoodDeg otherwise, try this hood first (e.g. the hub table's hood for this
   *     distance, so the arc looks like a normal hub shot); if it cannot make the shot, or is NaN,
   *     search the hood range for the lowest flywheel speed that does
   */
  public Solution solve(
      double horizontalDistanceMeters,
      double heightAboveLaunchMeters,
      double minHoodDeg,
      double maxHoodDeg,
      double maxRps,
      double hoodOverrideDeg,
      double preferredHoodDeg,
      double hoodStepDeg) {
    if (!calibrated) {
      return Solution.invalid("shot model not calibrated (shot table too small?)");
    }
    if (!(horizontalDistanceMeters > 0.0) || !Double.isFinite(heightAboveLaunchMeters)) {
      return Solution.invalid("bad target geometry");
    }

    if (hoodOverrideDeg >= minHoodDeg && hoodOverrideDeg <= maxHoodDeg) {
      Solution s = solveForHood(horizontalDistanceMeters, heightAboveLaunchMeters, hoodOverrideDeg);
      if (s.valid() && s.flywheelRps() > maxRps) {
        return Solution.invalid(String.format("needs %.1f rps > max", s.flywheelRps()));
      }
      return s;
    }

    if (Double.isFinite(preferredHoodDeg)) {
      double hood = Math.max(minHoodDeg, Math.min(maxHoodDeg, preferredHoodDeg));
      Solution s = solveForHood(horizontalDistanceMeters, heightAboveLaunchMeters, hood);
      if (s.valid() && s.flywheelRps() <= maxRps) {
        return s;
      }
    }

    Solution best = null;
    for (double hood = minHoodDeg; hood <= maxHoodDeg + 1e-9; hood += hoodStepDeg) {
      Solution s = solveForHood(horizontalDistanceMeters, heightAboveLaunchMeters, hood);
      if (s.valid()
          && s.flywheelRps() <= maxRps
          && (best == null || s.flywheelRps() < best.flywheelRps())) {
        best = s;
      }
    }
    return best != null ? best : Solution.invalid("target out of range for hood/flywheel limits");
  }

  Solution solveForHood(double distance, double dz, double hoodDeg) {
    double theta = Math.toRadians(launchAngleDegForHood(hoodDeg));
    double cos = Math.cos(theta);
    // From z = x tan(theta) - g x^2 / (2 v^2 cos^2(theta)), solved for v.
    double denom = 2.0 * cos * cos * (distance * Math.tan(theta) - dz);
    if (cos <= 0.0 || denom <= 0.0) {
      return Solution.invalid("launch angle too low to reach target height");
    }
    double speed = Math.sqrt(GRAVITY * distance * distance / denom);
    double rps = rpsForExitSpeed(speed);
    if (!(rps > 0.0)) {
      return Solution.invalid("required speed below model range");
    }
    double tof = distance / (speed * cos);
    return new Solution(true, hoodDeg, rps, tof, "");
  }
}
