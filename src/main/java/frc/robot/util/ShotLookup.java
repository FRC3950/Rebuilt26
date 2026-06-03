package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public final class ShotLookup {
  private final ShotTable2d table2d;
  private final List<Distancer.Row> legacyRows;
  private final InterpolatingTreeMap<Double, Distancer> legacyMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Distancer::interpolate);

  private ShotLookup(ShotTable2d table2d, List<Distancer.Row> legacyRows) {
    this.table2d = table2d;
    this.legacyRows = List.copyOf(legacyRows);
    for (Distancer.Row row : this.legacyRows) {
      legacyMap.put(row.d, Distancer.fromRow(row));
    }
  }

  public static ShotLookup fromDeploy() {
    return fromDeploy(ShotTableTarget.HUB);
  }

  public static ShotLookup fromDeploy(ShotTableTarget target) {
    if (target == ShotTableTarget.HUB) {
      return fromDeploy(target.deployFilename(), true);
    }
    return fromDeploy(target.deployFilename(), false);
  }

  public static ShotLookup fromDeploy(String filename) {
    return fromDeploy(filename, false);
  }

  private static ShotLookup fromDeploy(String filename, boolean allowLegacyFallback) {
    return ShotTable2d.loadFromDeploy(filename)
        .map(ShotLookup::fromShotTable2d)
        .orElseGet(
            () ->
                allowLegacyFallback
                    ? fromLegacyRows(Distancer.loadRowsFromDeploy("shot_table.json"))
                    : fromLegacyRows(List.of()));
  }

  public static ShotLookup fromShotTable2d(ShotTable2d table) {
    return new ShotLookup(table, List.of());
  }

  public static ShotLookup fromLegacyRows(List<Distancer.Row> rows) {
    ArrayList<Distancer.Row> sortedRows = new ArrayList<>(rows);
    sortedRows.sort(Comparator.comparingDouble(row -> row.d));
    return new ShotLookup(null, sortedRows);
  }

  public boolean isEmpty() {
    return table2d == null && legacyRows.isEmpty();
  }

  public boolean isTwoDimensional() {
    return table2d != null;
  }

  public Distancer getShot(double distanceMeters, double radialVelocityMetersPerSecond) {
    if (table2d != null) {
      return table2d.interpolate(distanceMeters, radialVelocityMetersPerSecond);
    }
    return getLegacyShot(distanceMeters);
  }

  public Distancer getShotClamped(double distanceMeters, double radialVelocityMetersPerSecond) {
    if (table2d != null) {
      return table2d.interpolateClamped(distanceMeters, radialVelocityMetersPerSecond);
    }
    return getLegacyShot(distanceMeters);
  }

  private Distancer getLegacyShot(double distanceMeters) {
    if (legacyRows.isEmpty()) {
      return null;
    }
    if (legacyRows.size() == 1) {
      return Distancer.fromRow(legacyRows.get(0));
    }

    double minDistance = legacyRows.get(0).d;
    double maxDistance = legacyRows.get(legacyRows.size() - 1).d;
    if (distanceMeters < minDistance) {
      return interpolateBetweenRows(legacyRows.get(0), legacyRows.get(1), distanceMeters);
    }
    if (distanceMeters > maxDistance) {
      return interpolateBetweenRows(
          legacyRows.get(legacyRows.size() - 2),
          legacyRows.get(legacyRows.size() - 1),
          distanceMeters);
    }
    return legacyMap.get(distanceMeters);
  }

  private static Distancer interpolateBetweenRows(
      Distancer.Row start, Distancer.Row end, double distanceMeters) {
    double t = (distanceMeters - start.d) / (end.d - start.d);
    return Distancer.fromRow(start).interpolate(Distancer.fromRow(end), t);
  }
}
