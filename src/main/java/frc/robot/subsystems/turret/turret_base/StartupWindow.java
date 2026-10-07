package frc.robot.subsystems.turret.turret_base;

/** Resolves sensor turns only under the operator-confirmed physical startup-window assumption. */
public final class StartupWindow {
  private StartupWindow() {}

  /** Returns sensor rotations, or NaN when no unambiguous in-window candidate is legal. */
  public static double selectSensorRotations(
      double absoluteRotations,
      double centerDeg,
      double halfWindowDeg,
      double sensorToTurretRatio,
      double minDeg,
      double maxDeg) {
    if (!Double.isFinite(absoluteRotations)
        || !Double.isFinite(centerDeg)
        || !Double.isFinite(halfWindowDeg)
        || !Double.isFinite(sensorToTurretRatio)
        || !Double.isFinite(minDeg)
        || !Double.isFinite(maxDeg)
        || sensorToTurretRatio <= 0.0
        || halfWindowDeg <= 0.0
        || halfWindowDeg >= 180.0 / sensorToTurretRatio
        || centerDeg - halfWindowDeg < minDeg
        || centerDeg + halfWindowDeg > maxDeg) {
      return Double.NaN;
    }
    double centerRotations = centerDeg / 360.0 * sensorToTurretRatio;
    double candidate = absoluteRotations + Math.rint(centerRotations - absoluteRotations);
    double angleDeg = candidate / sensorToTurretRatio * 360.0;
    return Math.abs(angleDeg - centerDeg) <= halfWindowDeg + 1e-9 ? candidate : Double.NaN;
  }
}
