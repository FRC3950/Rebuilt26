package frc.robot.util;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.Supplier;

/** Field-zone utilities for containment and simple predictive intersection checks. */
public final class Zones {
  private Zones() {}

  public interface Zone {
    boolean contains(Translation2d point);

    default Trigger contains(Supplier<Translation2d> pointSupplier) {
      return new Trigger(() -> contains(pointSupplier.get()));
    }
  }

  public interface PredictiveZone extends Zone {
    boolean willContain(Translation2d point, ChassisSpeeds fieldSpeeds, double dtSec);

    default Trigger willContain(
        Supplier<Translation2d> pointSupplier,
        Supplier<ChassisSpeeds> fieldSpeedsSupplier,
        double dtSec) {
      return new Trigger(() -> willContain(pointSupplier.get(), fieldSpeedsSupplier.get(), dtSec));
    }
  }

  /** Axis-aligned rectangular zone with predictive swept-segment intersection check. */
  public static class PredictiveRectZone implements PredictiveZone {
    private final Rectangle2d rectangle;
    private final double xMin;
    private final double xMax;
    private final double yMin;
    private final double yMax;

    public PredictiveRectZone(double xMin, double xMax, double yMin, double yMax) {
      this(new Rectangle2d(new Translation2d(xMin, yMin), new Translation2d(xMax, yMax)));
    }

    public PredictiveRectZone(Rectangle2d rectangle) {
      this.rectangle = rectangle;

      Translation2d center = rectangle.getCenter().getTranslation();
      double halfWidth = rectangle.getXWidth() / 2.0;
      double halfHeight = rectangle.getYWidth() / 2.0;
      this.xMin = center.getX() - halfWidth;
      this.xMax = center.getX() + halfWidth;
      this.yMin = center.getY() - halfHeight;
      this.yMax = center.getY() + halfHeight;
    }

    @Override
    public boolean contains(Translation2d point) {
      return rectangle.contains(point);
    }

    @Override
    public boolean willContain(Translation2d point, ChassisSpeeds fieldSpeeds, double dtSec) {
      double lookaheadSec = Math.max(0.0, dtSec);
      Translation2d predictedPoint =
          point.plus(
              new Translation2d(
                  fieldSpeeds.vxMetersPerSecond * lookaheadSec,
                  fieldSpeeds.vyMetersPerSecond * lookaheadSec));

      return contains(point)
          || contains(predictedPoint)
          || segmentIntersectsAabb(point, predictedPoint, xMin, xMax, yMin, yMax);
    }

    public double getYCenter() {
      return (yMin + yMax) / 2.0;
    }
  }

  public static class PredictiveZoneCollection implements PredictiveZone {
    private final PredictiveRectZone[] zones;

    public PredictiveZoneCollection(PredictiveRectZone... zones) {
      this.zones = zones;
    }

    @Override
    public boolean contains(Translation2d point) {
      for (PredictiveRectZone zone : zones) {
        if (zone.contains(point)) {
          return true;
        }
      }
      return false;
    }

    @Override
    public boolean willContain(Translation2d point, ChassisSpeeds fieldSpeeds, double dtSec) {
      for (PredictiveRectZone zone : zones) {
        if (zone.willContain(point, fieldSpeeds, dtSec)) {
          return true;
        }
      }
      return false;
    }

    public Optional<PredictiveRectZone> firstWillContain(
        Translation2d point, ChassisSpeeds fieldSpeeds, double dtSec) {
      for (PredictiveRectZone zone : zones) {
        if (zone.willContain(point, fieldSpeeds, dtSec)) {
          return Optional.of(zone);
        }
      }
      return Optional.empty();
    }
  }

  private static final PredictiveRectZone TRENCH_BOTTOM_LEFT =
      new PredictiveRectZone(4.0287268168, 5.2225317698, 0.0, 1.2850851616);
  private static final PredictiveRectZone TRENCH_BOTTOM_RIGHT =
      new PredictiveRectZone(11.318528544, 12.5123282646, 0.0, 1.2850851616);
  private static final PredictiveRectZone TRENCH_TOP_LEFT =
      new PredictiveRectZone(4.0287268168, 5.2225317698, 6.7968819866, 8.0756125);
  private static final PredictiveRectZone TRENCH_TOP_RIGHT =
      new PredictiveRectZone(11.318528544, 12.5123282646, 6.7968819866, 8.0756125);

  public static final PredictiveZoneCollection TRENCH_ZONES =
      new PredictiveZoneCollection(
          TRENCH_BOTTOM_LEFT, TRENCH_BOTTOM_RIGHT, TRENCH_TOP_LEFT, TRENCH_TOP_RIGHT);

  private static boolean segmentIntersectsAabb(
      Translation2d p0, Translation2d p1, double xMin, double xMax, double yMin, double yMax) {
    double t0 = 0.0;
    double t1 = 1.0;

    double dx = p1.getX() - p0.getX();
    double dy = p1.getY() - p0.getY();

    double[] p = {-dx, dx, -dy, dy};
    double[] q = {p0.getX() - xMin, xMax - p0.getX(), p0.getY() - yMin, yMax - p0.getY()};

    for (int i = 0; i < 4; i++) {
      if (Math.abs(p[i]) < 1e-9) {
        if (q[i] < 0.0) {
          return false;
        }
      } else {
        double t = q[i] / p[i];
        if (p[i] < 0.0) {
          t0 = Math.max(t0, t);
        } else {
          t1 = Math.min(t1, t);
        }
        if (t0 > t1) {
          return false;
        }
      }
    }
    return true;
  }
}
