package frc.robot.shotmap;

import frc.robot.util.ShotTable2d;
import frc.robot.util.ShotTableTarget;
import java.util.ArrayList;
import java.util.List;

public class ShotMapGenerator {
  private static final double FERRY_ACCEPTABLE_LANDING_ERROR_METERS = 0.15;

  private final ShotMapConfig config;
  private final HubShotPhysics physics;

  public ShotMapGenerator(ShotMapConfig config) {
    this.config = config;
    this.physics = new HubShotPhysics(config.physicsParameters());
  }

  public Output generate() {
    double[] distances =
        axis(config.minDistanceMeters(), config.maxDistanceMeters(), config.distanceStepMeters());
    double[] radialVelocities =
        axis(
            config.minRadialVelocityMetersPerSecond(),
            config.maxRadialVelocityMetersPerSecond(),
            config.radialVelocityStepMetersPerSecond());
    ShotTable2d.Sample[][] samples =
        new ShotTable2d.Sample[distances.length][radialVelocities.length];
    ArrayList<DiagnosticRow> diagnostics = new ArrayList<>();

    for (int d = 0; d < distances.length; d++) {
      for (int v = 0; v < radialVelocities.length; v++) {
        Candidate selected = selectShot(distances[d], radialVelocities[v]);
        samples[d][v] =
            new ShotTable2d.Sample(
                selected.hoodDeg(),
                selected.flywheelRps(),
                selected.tofSec(),
                selected.robustnessMarginMeters());
        diagnostics.add(
            new DiagnosticRow(
                distances[d],
                radialVelocities[v],
                selected.hoodDeg(),
                selected.flywheelRps(),
                selected.tofSec(),
                selected.robustnessMarginMeters(),
                selected.postBounceHorizontalVelocityMetersPerSecond(),
                selected.maxHeightMeters(),
                selected.firstContactXFieldMeters(),
                selected.firstContactYFieldMeters(),
                selected.firstContactSurfaceHeightMeters(),
                selected.firstContactOnBump(),
                selected.baseScored()));
      }
    }

    return new Output(
        new ShotTable2d(distances, radialVelocities, samples), List.copyOf(diagnostics));
  }

  private Candidate selectShot(double distanceMeters, double radialVelocityMetersPerSecond) {
    Candidate coarse = search(config.coarseSearch(), distanceMeters, radialVelocityMetersPerSecond);
    ShotMapConfig.SearchRange refinedRange =
        new ShotMapConfig.SearchRange(
            clamp(
                coarse.hoodDeg() - config.refinement().hoodWindowDeg(),
                config.coarseSearch().minHoodDeg(),
                config.coarseSearch().maxHoodDeg()),
            clamp(
                coarse.hoodDeg() + config.refinement().hoodWindowDeg(),
                config.coarseSearch().minHoodDeg(),
                config.coarseSearch().maxHoodDeg()),
            config.refinement().hoodStepDeg(),
            clamp(
                coarse.flywheelRps() - config.refinement().flywheelWindowRps(),
                config.coarseSearch().minFlywheelRps(),
                config.coarseSearch().maxFlywheelRps()),
            clamp(
                coarse.flywheelRps() + config.refinement().flywheelWindowRps(),
                config.coarseSearch().minFlywheelRps(),
                config.coarseSearch().maxFlywheelRps()),
            config.refinement().flywheelStepRps());
    return search(refinedRange, distanceMeters, radialVelocityMetersPerSecond);
  }

  private Candidate search(
      ShotMapConfig.SearchRange range,
      double distanceMeters,
      double radialVelocityMetersPerSecond) {
    Candidate best = null;
    for (double hoodDeg = range.minHoodDeg();
        hoodDeg <= range.maxHoodDeg() + 1e-9;
        hoodDeg += range.hoodStepDeg()) {
      for (double flywheelRps = range.minFlywheelRps();
          flywheelRps <= range.maxFlywheelRps() + 1e-9;
          flywheelRps += range.flywheelStepRps()) {
        Candidate candidate =
            evaluateCandidate(
                distanceMeters, radialVelocityMetersPerSecond, hoodDeg, flywheelRps, best);
        if (best == null || candidate.compareTo(best) > 0) {
          best = candidate;
        }
      }
    }
    return best;
  }

  private Candidate evaluateCandidate(
      double distanceMeters,
      double radialVelocityMetersPerSecond,
      double hoodDeg,
      double flywheelRps,
      Candidate bestCandidate) {
    HubShotPhysics.Result base =
        evaluateShot(distanceMeters, radialVelocityMetersPerSecond, hoodDeg, flywheelRps);
    if (config.target() != ShotTableTarget.FERRY
        && bestCandidate != null
        && base.marginMeters() <= bestCandidate.robustnessMarginMeters()) {
      return new Candidate(
          hoodDeg,
          flywheelRps,
          base.tofSec(),
          base.marginMeters(),
          ferryLandingErrorMeters(base.marginMeters()),
          base.marginMeters(),
          base.postBounceHorizontalVelocityMps(),
          base.maxHeightMeters(),
          base.firstContactXFieldMeters(),
          base.firstContactYFieldMeters(),
          base.firstContactSurfaceHeightMeters(),
          base.firstContactOnBump(),
          ferryTofPreferenceError(distanceMeters, base.tofSec()),
          ferryRankingCost(
              distanceMeters, hoodDeg, flywheelRps, base.tofSec(), base.maxHeightMeters()),
          config.target() == ShotTableTarget.FERRY,
          base.scored());
    }

    double worstMargin = Double.POSITIVE_INFINITY;
    double[] hoodErrors = {
      -config.physicsParameters().hoodErrorDeg(), 0.0, config.physicsParameters().hoodErrorDeg()
    };
    double[] flywheelScales = {
      1.0 - config.physicsParameters().flywheelScaleError(),
      1.0,
      1.0 + config.physicsParameters().flywheelScaleError()
    };
    double[] distanceErrors = {
      -config.physicsParameters().distanceErrorMeters(),
      0.0,
      config.physicsParameters().distanceErrorMeters()
    };

    for (double hoodError : hoodErrors) {
      for (double flywheelScale : flywheelScales) {
        for (double distanceError : distanceErrors) {
          HubShotPhysics.Result perturbed =
              evaluateShot(
                  distanceMeters + distanceError,
                  radialVelocityMetersPerSecond,
                  hoodDeg + hoodError,
                  flywheelRps * flywheelScale);
          worstMargin = Math.min(worstMargin, perturbed.marginMeters());
        }
      }
    }

    return new Candidate(
        hoodDeg,
        flywheelRps,
        base.tofSec(),
        base.marginMeters(),
        ferryLandingErrorMeters(base.marginMeters()),
        worstMargin,
        base.postBounceHorizontalVelocityMps(),
        base.maxHeightMeters(),
        base.firstContactXFieldMeters(),
        base.firstContactYFieldMeters(),
        base.firstContactSurfaceHeightMeters(),
        base.firstContactOnBump(),
        ferryTofPreferenceError(distanceMeters, base.tofSec()),
        ferryRankingCost(
            distanceMeters, hoodDeg, flywheelRps, base.tofSec(), base.maxHeightMeters()),
        config.target() == ShotTableTarget.FERRY,
        base.scored());
  }

  private HubShotPhysics.Result evaluateShot(
      double distanceMeters,
      double radialVelocityMetersPerSecond,
      double hoodDeg,
      double flywheelRps) {
    return config.target() == ShotTableTarget.FERRY
        ? physics.evaluateFerryFirstTerrainContact(
            distanceMeters, radialVelocityMetersPerSecond, hoodDeg, flywheelRps)
        : physics.evaluate(distanceMeters, radialVelocityMetersPerSecond, hoodDeg, flywheelRps);
  }

  private double ferryLandingErrorMeters(double baseMarginMeters) {
    return config.physicsParameters().entryRadiusMeters() - baseMarginMeters;
  }

  private static double ferryTofPreferenceError(double distanceMeters, double tofSec) {
    double preferredTofSec = clamp(0.55 + 0.12 * distanceMeters, 0.75, 1.35);
    return Math.abs(tofSec - preferredTofSec);
  }

  private double ferryRankingCost(
      double distanceMeters,
      double hoodDeg,
      double flywheelRps,
      double tofSec,
      double maxHeightMeters) {
    double preferredHoodDeg = ferryPreferredHoodDeg(distanceMeters);
    double hoodErrorDeg = Math.abs(hoodDeg - preferredHoodDeg);
    double maxHeightErrorMeters = Math.abs(maxHeightMeters - 2.0);
    double excessHeightMeters = Math.max(0.0, maxHeightMeters - 2.2);
    return maxHeightErrorMeters * 7.0
        + excessHeightMeters * 30.0
        + tofSec * 4.0
        + hoodErrorDeg * 0.08
        + flywheelRps * 0.002;
  }

  private double ferryPreferredHoodDeg(double distanceMeters) {
    double preferred = 17.5 + distanceMeters * 1.15;
    return clamp(preferred, config.coarseSearch().minHoodDeg(), config.coarseSearch().maxHoodDeg());
  }

  private static double[] axis(double min, double max, double step) {
    if (step <= 0.0) {
      throw new IllegalArgumentException("axis step must be positive");
    }
    ArrayList<Double> values = new ArrayList<>();
    for (double value = min; value <= max + 1e-9; value += step) {
      values.add(round(value));
    }
    return values.stream().mapToDouble(Double::doubleValue).toArray();
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  private static double round(double value) {
    return Math.rint(value * 1_000_000.0) / 1_000_000.0;
  }

  public record Output(ShotTable2d table, List<DiagnosticRow> diagnostics) {}

  public record DiagnosticRow(
      double distanceMeters,
      double radialVelocityMetersPerSecond,
      double hoodDeg,
      double flywheelRps,
      double tofSec,
      double robustnessMarginMeters,
      double postBounceHorizontalVelocityMetersPerSecond,
      double maxHeightMeters,
      double firstContactXFieldMeters,
      double firstContactYFieldMeters,
      double firstContactSurfaceHeightMeters,
      boolean firstContactOnBump,
      boolean baseScored) {}

  private record Candidate(
      double hoodDeg,
      double flywheelRps,
      double tofSec,
      double baseMarginMeters,
      double baseLandingErrorMeters,
      double robustnessMarginMeters,
      double postBounceHorizontalVelocityMetersPerSecond,
      double maxHeightMeters,
      double firstContactXFieldMeters,
      double firstContactYFieldMeters,
      double firstContactSurfaceHeightMeters,
      boolean firstContactOnBump,
      double tofPreferenceErrorSec,
      double ferryRankingCost,
      boolean ferryCandidate,
      boolean baseScored)
      implements Comparable<Candidate> {
    @Override
    public int compareTo(Candidate other) {
      if (ferryCandidate) {
        boolean thisValidFerryLanding =
            baseLandingErrorMeters <= FERRY_ACCEPTABLE_LANDING_ERROR_METERS && !firstContactOnBump;
        boolean otherValidFerryLanding =
            other.baseLandingErrorMeters <= FERRY_ACCEPTABLE_LANDING_ERROR_METERS
                && !other.firstContactOnBump;
        if (thisValidFerryLanding != otherValidFerryLanding) {
          return thisValidFerryLanding ? 1 : -1;
        }

        if (!thisValidFerryLanding) {
          if (firstContactOnBump != other.firstContactOnBump) {
            return firstContactOnBump ? -1 : 1;
          }
          int landingErrorComparison =
              Double.compare(other.baseLandingErrorMeters, this.baseLandingErrorMeters);
          if (landingErrorComparison != 0) {
            return landingErrorComparison;
          }
        }

        boolean thisLowEnough = maxHeightMeters <= 2.2;
        boolean otherLowEnough = other.maxHeightMeters <= 2.2;
        if (thisLowEnough != otherLowEnough) {
          return thisLowEnough ? 1 : -1;
        }
        int rankingComparison = Double.compare(other.ferryRankingCost, this.ferryRankingCost);
        if (rankingComparison != 0) {
          return rankingComparison;
        }
        int robustnessComparison =
            Double.compare(this.robustnessMarginMeters, other.robustnessMarginMeters);
        if (robustnessComparison != 0) {
          return robustnessComparison;
        }
        return Double.compare(this.baseMarginMeters, other.baseMarginMeters);
      }
      int marginComparison =
          Double.compare(this.robustnessMarginMeters, other.robustnessMarginMeters);
      if (marginComparison != 0) {
        return marginComparison;
      }
      return Double.compare(other.tofSec, this.tofSec);
    }
  }
}
