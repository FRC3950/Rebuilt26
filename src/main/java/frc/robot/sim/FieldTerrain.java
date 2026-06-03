package frc.robot.sim;

import edu.wpi.first.math.geometry.Translation2d;

/** Shared REBUILT field terrain model for bump-aware simulation and shot generation. */
public final class FieldTerrain {
  private static final double EPSILON = 1e-9;

  private FieldTerrain() {}

  public static double getSurfaceHeightMeters(Translation2d position) {
    if (!isBumpFootprint(position)) {
      return 0.0;
    }

    if (position.getX() <= FuelSim.BLUE_BUMP_MAX_X) {
      return bumpHeightAtX(position.getX(), FuelSim.BLUE_BUMP_PEAK_X);
    }
    return bumpHeightAtX(position.getX(), FuelSim.RED_BUMP_PEAK_X);
  }

  public static boolean isBumpFootprint(Translation2d position) {
    return isInBumpYBand(position.getY())
        && (isWithin(position.getX(), FuelSim.BLUE_BUMP_MIN_X, FuelSim.BLUE_BUMP_MAX_X)
            || isWithin(position.getX(), FuelSim.RED_BUMP_MIN_X, FuelSim.RED_BUMP_MAX_X));
  }

  public static boolean isBlueBumpFootprint(Translation2d position) {
    return isInBumpYBand(position.getY())
        && isWithin(position.getX(), FuelSim.BLUE_BUMP_MIN_X, FuelSim.BLUE_BUMP_MAX_X);
  }

  public static boolean isRedBumpFootprint(Translation2d position) {
    return isInBumpYBand(position.getY())
        && isWithin(position.getX(), FuelSim.RED_BUMP_MIN_X, FuelSim.RED_BUMP_MAX_X);
  }

  public static double blueBumpMinXMeters() {
    return FuelSim.BLUE_BUMP_MIN_X;
  }

  public static double blueBumpMaxXMeters() {
    return FuelSim.BLUE_BUMP_MAX_X;
  }

  public static double bumpHeightMeters() {
    return FuelSim.BUMP_HEIGHT;
  }

  private static double bumpHeightAtX(double xMeters, double peakXMeters) {
    double distanceFromPeak = Math.abs(xMeters - peakXMeters);
    double normalized = 1.0 - distanceFromPeak / FuelSim.BUMP_HALF_LENGTH;
    return FuelSim.BUMP_HEIGHT * Math.max(0.0, normalized);
  }

  private static boolean isInBumpYBand(double yMeters) {
    return isWithin(yMeters, FuelSim.LOWER_BUMP_MIN_Y, FuelSim.LOWER_BUMP_MAX_Y)
        || isWithin(yMeters, FuelSim.UPPER_BUMP_MIN_Y, FuelSim.UPPER_BUMP_MAX_Y);
  }

  private static boolean isWithin(double value, double min, double max) {
    return value >= min - EPSILON && value <= max + EPSILON;
  }
}
