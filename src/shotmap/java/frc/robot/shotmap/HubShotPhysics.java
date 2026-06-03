package frc.robot.shotmap;

import static frc.robot.Constants.SimConstants.Fuel.EFFECTIVE_FLYWHEEL_RADIUS_METERS;
import static frc.robot.Constants.SimConstants.Fuel.SHOTMAP_EXIT_VELOCITY_SCALE;
import static frc.robot.Constants.SimConstants.Fuel.SHOTMAP_LAUNCH_ANGLE_OFFSET_DEG;
import static frc.robot.Constants.SimConstants.Fuel.TURRET_LAUNCH_HEIGHT_METERS;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.sim.FieldTerrain;

public class HubShotPhysics {
  private static final double DRAG_FORCE_FACTOR = 0.5 * 1.2041 * 0.47 * Math.PI * 0.075 * 0.075;
  private static final double FUEL_MASS_KG = 0.448 * 0.45392;
  private static final double NUMERICAL_DT_SEC = 0.01;
  private static final double FUEL_RADIUS_METERS = 0.075;
  private static final double FIELD_COEFFICIENT_OF_RESTITUTION = Math.sqrt(11.0 / 51.5);
  private static final double MISSED_MAX_HEIGHT_METERS = Double.POSITIVE_INFINITY;
  private static final double BLUE_LEFT_FERRY_TARGET_X_METERS = FieldConstants.hubGuy.getX() - 2.5;
  private static final double BLUE_LEFT_FERRY_TARGET_Y_METERS = FieldConstants.hubGuy.getY() + 1.8;

  private final Parameters parameters;

  public HubShotPhysics() {
    this(Parameters.rebuiltHub());
  }

  public HubShotPhysics(Parameters parameters) {
    this.parameters = parameters;
  }

  public Result evaluate(
      double distanceMeters,
      double radialVelocityMetersPerSecond,
      double hoodDeg,
      double flywheelRps) {
    double hoodRadians = Math.toRadians(hoodDeg + parameters.launchAngleOffsetDeg());
    double exitSpeedMetersPerSecond =
        2.0
            * Math.PI
            * parameters.effectiveFlywheelRadiusMeters()
            * flywheelRps
            * parameters.exitVelocityScale();
    double horizontalVelocityMetersPerSecond =
        exitSpeedMetersPerSecond * Math.cos(hoodRadians) + radialVelocityMetersPerSecond;
    double verticalVelocityMetersPerSecond = exitSpeedMetersPerSecond * Math.sin(hoodRadians);

    if (horizontalVelocityMetersPerSecond <= 0.0 || verticalVelocityMetersPerSecond <= 0.0) {
      return Result.missed(
          parameters.maxFlightTimeSec(),
          -Double.MAX_VALUE,
          Double.POSITIVE_INFINITY,
          MISSED_MAX_HEIGHT_METERS);
    }

    return parameters.simulateAirResistance()
        ? evaluateNumerically(
            distanceMeters, horizontalVelocityMetersPerSecond, verticalVelocityMetersPerSecond)
        : evaluateAnalytically(
            distanceMeters, horizontalVelocityMetersPerSecond, verticalVelocityMetersPerSecond);
  }

  public Result evaluateFirstBounce(
      double distanceMeters,
      double radialVelocityMetersPerSecond,
      double hoodDeg,
      double flywheelRps) {
    double hoodRadians = Math.toRadians(hoodDeg + parameters.launchAngleOffsetDeg());
    double exitSpeedMetersPerSecond =
        2.0
            * Math.PI
            * parameters.effectiveFlywheelRadiusMeters()
            * flywheelRps
            * parameters.exitVelocityScale();
    double horizontalVelocityMetersPerSecond =
        exitSpeedMetersPerSecond * Math.cos(hoodRadians) + radialVelocityMetersPerSecond;
    double verticalVelocityMetersPerSecond = exitSpeedMetersPerSecond * Math.sin(hoodRadians);

    if (horizontalVelocityMetersPerSecond <= 0.0 || verticalVelocityMetersPerSecond <= 0.0) {
      return Result.missed(
          parameters.maxFlightTimeSec(),
          -Double.MAX_VALUE,
          Double.POSITIVE_INFINITY,
          MISSED_MAX_HEIGHT_METERS);
    }

    return parameters.simulateAirResistance()
        ? evaluateFirstBounceNumerically(
            distanceMeters, horizontalVelocityMetersPerSecond, verticalVelocityMetersPerSecond)
        : evaluateFirstBounceAnalytically(
            distanceMeters, horizontalVelocityMetersPerSecond, verticalVelocityMetersPerSecond);
  }

  public Result evaluateFerryFirstTerrainContact(
      double distanceMeters,
      double radialVelocityMetersPerSecond,
      double hoodDeg,
      double flywheelRps) {
    double hoodRadians = Math.toRadians(hoodDeg + parameters.launchAngleOffsetDeg());
    double exitSpeedMetersPerSecond =
        2.0
            * Math.PI
            * parameters.effectiveFlywheelRadiusMeters()
            * flywheelRps
            * parameters.exitVelocityScale();
    double horizontalVelocityMetersPerSecond =
        exitSpeedMetersPerSecond * Math.cos(hoodRadians) + radialVelocityMetersPerSecond;
    double verticalVelocityMetersPerSecond = exitSpeedMetersPerSecond * Math.sin(hoodRadians);

    if (horizontalVelocityMetersPerSecond <= 0.0 || verticalVelocityMetersPerSecond <= 0.0) {
      return Result.missed(
          parameters.maxFlightTimeSec(),
          -Double.MAX_VALUE,
          Double.POSITIVE_INFINITY,
          MISSED_MAX_HEIGHT_METERS);
    }

    return evaluateFerryFirstTerrainContactNumerically(
        distanceMeters, horizontalVelocityMetersPerSecond, verticalVelocityMetersPerSecond);
  }

  private Result evaluateAnalytically(
      double distanceMeters,
      double horizontalVelocityMetersPerSecond,
      double verticalVelocityMetersPerSecond) {
    double tofSec;
    double launchHeightMeters = parameters.launchHeightMeters();
    double entryHeightMeters = parameters.entryHeightMeters();
    double gravityMetersPerSecondSquared = parameters.gravityMetersPerSecondSquared();

    if (Math.abs(gravityMetersPerSecondSquared) < 1e-9) {
      if (Math.abs(verticalVelocityMetersPerSecond) < 1e-9) {
        return Result.missed(
            parameters.maxFlightTimeSec(),
            -Double.MAX_VALUE,
            Double.POSITIVE_INFINITY,
            MISSED_MAX_HEIGHT_METERS);
      }
      tofSec = (entryHeightMeters - launchHeightMeters) / verticalVelocityMetersPerSecond;
    } else {
      double a = -0.5 * gravityMetersPerSecondSquared;
      double b = verticalVelocityMetersPerSecond;
      double c = launchHeightMeters - entryHeightMeters;
      double discriminant = b * b - 4.0 * a * c;
      if (discriminant < 0.0) {
        return Result.missed(
            parameters.maxFlightTimeSec(),
            -Double.MAX_VALUE,
            Double.POSITIVE_INFINITY,
            MISSED_MAX_HEIGHT_METERS);
      }
      double rootA = (-b + Math.sqrt(discriminant)) / (2.0 * a);
      double rootB = (-b - Math.sqrt(discriminant)) / (2.0 * a);
      tofSec = Math.max(rootA, rootB);
    }

    if (!Double.isFinite(tofSec) || tofSec <= 0.0 || tofSec > parameters.maxFlightTimeSec()) {
      return Result.missed(
          parameters.maxFlightTimeSec(),
          -Double.MAX_VALUE,
          Double.POSITIVE_INFINITY,
          MISSED_MAX_HEIGHT_METERS);
    }

    double xAtEntryMeters = horizontalVelocityMetersPerSecond * tofSec;
    double marginMeters =
        parameters.entryRadiusMeters() - Math.abs(xAtEntryMeters - distanceMeters);
    double maxHeightMeters =
        getAnalyticalMaxHeightMeters(
            verticalVelocityMetersPerSecond, parameters.launchHeightMeters());
    return new Result(
        marginMeters >= 0.0,
        tofSec,
        marginMeters,
        Math.abs(horizontalVelocityMetersPerSecond),
        maxHeightMeters,
        Double.NaN,
        Double.NaN,
        Double.NaN,
        false);
  }

  private Result evaluateFirstBounceAnalytically(
      double distanceMeters,
      double horizontalVelocityMetersPerSecond,
      double verticalVelocityMetersPerSecond) {
    double a = -0.5 * parameters.gravityMetersPerSecondSquared();
    double b = verticalVelocityMetersPerSecond;
    double c = parameters.launchHeightMeters() - FUEL_RADIUS_METERS;
    double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0) {
      return Result.missed(
          parameters.maxFlightTimeSec(),
          -Double.MAX_VALUE,
          Double.POSITIVE_INFINITY,
          MISSED_MAX_HEIGHT_METERS);
    }

    double rootA = (-b + Math.sqrt(discriminant)) / (2.0 * a);
    double rootB = (-b - Math.sqrt(discriminant)) / (2.0 * a);
    double tofSec = Math.max(rootA, rootB);
    if (!Double.isFinite(tofSec) || tofSec <= 0.0 || tofSec > parameters.maxFlightTimeSec()) {
      return Result.missed(
          parameters.maxFlightTimeSec(),
          -Double.MAX_VALUE,
          Double.POSITIVE_INFINITY,
          MISSED_MAX_HEIGHT_METERS);
    }

    double xAtBounceMeters = horizontalVelocityMetersPerSecond * tofSec;
    double marginMeters =
        parameters.entryRadiusMeters() - Math.abs(xAtBounceMeters - distanceMeters);
    double maxHeightMeters =
        getAnalyticalMaxHeightMeters(
            verticalVelocityMetersPerSecond, parameters.launchHeightMeters());
    return new Result(
        marginMeters >= 0.0,
        tofSec,
        marginMeters,
        Math.abs(horizontalVelocityMetersPerSecond) * FIELD_COEFFICIENT_OF_RESTITUTION,
        maxHeightMeters,
        Double.NaN,
        Double.NaN,
        Double.NaN,
        false);
  }

  private Result evaluateNumerically(
      double distanceMeters,
      double horizontalVelocityMetersPerSecond,
      double verticalVelocityMetersPerSecond) {
    double xMeters = 0.0;
    double zMeters = parameters.launchHeightMeters();
    double vxMetersPerSecond = horizontalVelocityMetersPerSecond;
    double vzMetersPerSecond = verticalVelocityMetersPerSecond;
    double lastX = xMeters;
    double lastZ = zMeters;
    double maxHeightMeters = zMeters;

    for (double tSec = 0.0; tSec <= parameters.maxFlightTimeSec(); tSec += NUMERICAL_DT_SEC) {
      lastX = xMeters;
      lastZ = zMeters;
      xMeters += vxMetersPerSecond * NUMERICAL_DT_SEC;
      zMeters += vzMetersPerSecond * NUMERICAL_DT_SEC;
      maxHeightMeters = Math.max(maxHeightMeters, zMeters);

      double speed = Math.hypot(vxMetersPerSecond, vzMetersPerSecond);
      double dragAcceleration = DRAG_FORCE_FACTOR * speed / FUEL_MASS_KG;
      vxMetersPerSecond -= vxMetersPerSecond * dragAcceleration * NUMERICAL_DT_SEC;
      vzMetersPerSecond -=
          (parameters.gravityMetersPerSecondSquared() + vzMetersPerSecond * dragAcceleration)
              * NUMERICAL_DT_SEC;

      boolean crossedDownward =
          zMeters <= parameters.entryHeightMeters() && lastZ > parameters.entryHeightMeters();
      if (crossedDownward) {
        double zSpan = zMeters - lastZ;
        double crossingT =
            Math.abs(zSpan) < 1e-9 ? 1.0 : (parameters.entryHeightMeters() - lastZ) / zSpan;
        double xAtEntryMeters = lastX + (xMeters - lastX) * crossingT;
        double tofSec = tSec + NUMERICAL_DT_SEC * crossingT;
        double marginMeters =
            parameters.entryRadiusMeters() - Math.abs(xAtEntryMeters - distanceMeters);
        return new Result(
            marginMeters >= 0.0,
            tofSec,
            marginMeters,
            Math.abs(vxMetersPerSecond),
            maxHeightMeters,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            false);
      }
    }

    return Result.missed(
        parameters.maxFlightTimeSec(),
        -Double.MAX_VALUE,
        Double.POSITIVE_INFINITY,
        MISSED_MAX_HEIGHT_METERS);
  }

  private Result evaluateFirstBounceNumerically(
      double distanceMeters,
      double horizontalVelocityMetersPerSecond,
      double verticalVelocityMetersPerSecond) {
    double xMeters = 0.0;
    double zMeters = parameters.launchHeightMeters();
    double vxMetersPerSecond = horizontalVelocityMetersPerSecond;
    double vzMetersPerSecond = verticalVelocityMetersPerSecond;
    double lastX = xMeters;
    double lastZ = zMeters;
    double lastVx = vxMetersPerSecond;
    double maxHeightMeters = zMeters;

    for (double tSec = 0.0; tSec <= parameters.maxFlightTimeSec(); tSec += NUMERICAL_DT_SEC) {
      lastX = xMeters;
      lastZ = zMeters;
      lastVx = vxMetersPerSecond;
      xMeters += vxMetersPerSecond * NUMERICAL_DT_SEC;
      zMeters += vzMetersPerSecond * NUMERICAL_DT_SEC;
      maxHeightMeters = Math.max(maxHeightMeters, zMeters);

      double speed = Math.hypot(vxMetersPerSecond, vzMetersPerSecond);
      double dragAcceleration = DRAG_FORCE_FACTOR * speed / FUEL_MASS_KG;
      vxMetersPerSecond -= vxMetersPerSecond * dragAcceleration * NUMERICAL_DT_SEC;
      vzMetersPerSecond -=
          (parameters.gravityMetersPerSecondSquared() + vzMetersPerSecond * dragAcceleration)
              * NUMERICAL_DT_SEC;

      boolean crossedFloor = zMeters <= FUEL_RADIUS_METERS && lastZ > FUEL_RADIUS_METERS;
      if (crossedFloor) {
        double zSpan = zMeters - lastZ;
        double crossingT = Math.abs(zSpan) < 1e-9 ? 1.0 : (FUEL_RADIUS_METERS - lastZ) / zSpan;
        double xAtBounceMeters = lastX + (xMeters - lastX) * crossingT;
        double vxAtBounceMetersPerSecond =
            lastVx + (vxMetersPerSecond - lastVx) * Math.max(0.0, Math.min(1.0, crossingT));
        double tofSec = tSec + NUMERICAL_DT_SEC * crossingT;
        double marginMeters =
            parameters.entryRadiusMeters() - Math.abs(xAtBounceMeters - distanceMeters);
        return new Result(
            marginMeters >= 0.0,
            tofSec,
            marginMeters,
            Math.abs(vxAtBounceMetersPerSecond) * FIELD_COEFFICIENT_OF_RESTITUTION,
            maxHeightMeters,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            false);
      }
    }

    return Result.missed(
        parameters.maxFlightTimeSec(),
        -Double.MAX_VALUE,
        Double.POSITIVE_INFINITY,
        MISSED_MAX_HEIGHT_METERS);
  }

  private Result evaluateFerryFirstTerrainContactNumerically(
      double distanceMeters,
      double horizontalVelocityMetersPerSecond,
      double verticalVelocityMetersPerSecond) {
    double xMeters = 0.0;
    double zMeters = parameters.launchHeightMeters();
    double vxMetersPerSecond = horizontalVelocityMetersPerSecond;
    double vzMetersPerSecond = verticalVelocityMetersPerSecond;
    double lastX = xMeters;
    double lastZ = zMeters;
    double lastVx = vxMetersPerSecond;
    double maxHeightMeters = zMeters;
    double lastClearanceMeters =
        zMeters - getFerryTerrainSurfaceHeightMeters(distanceMeters, xMeters) - FUEL_RADIUS_METERS;

    for (double tSec = 0.0; tSec <= parameters.maxFlightTimeSec(); tSec += NUMERICAL_DT_SEC) {
      lastX = xMeters;
      lastZ = zMeters;
      lastVx = vxMetersPerSecond;
      xMeters += vxMetersPerSecond * NUMERICAL_DT_SEC;
      zMeters += vzMetersPerSecond * NUMERICAL_DT_SEC;
      maxHeightMeters = Math.max(maxHeightMeters, zMeters);

      double speed = Math.hypot(vxMetersPerSecond, vzMetersPerSecond);
      if (parameters.simulateAirResistance()) {
        double dragAcceleration = DRAG_FORCE_FACTOR * speed / FUEL_MASS_KG;
        vxMetersPerSecond -= vxMetersPerSecond * dragAcceleration * NUMERICAL_DT_SEC;
        vzMetersPerSecond -=
            (parameters.gravityMetersPerSecondSquared() + vzMetersPerSecond * dragAcceleration)
                * NUMERICAL_DT_SEC;
      } else {
        vzMetersPerSecond -= parameters.gravityMetersPerSecondSquared() * NUMERICAL_DT_SEC;
      }

      double clearanceMeters =
          zMeters
              - getFerryTerrainSurfaceHeightMeters(distanceMeters, xMeters)
              - FUEL_RADIUS_METERS;
      if (clearanceMeters <= 0.0 && lastClearanceMeters > 0.0) {
        double crossingT =
            lastClearanceMeters / Math.max(1e-9, lastClearanceMeters - clearanceMeters);
        crossingT = Math.max(0.0, Math.min(1.0, crossingT));
        double xAtContactMeters = lastX + (xMeters - lastX) * crossingT;
        double vxAtContactMetersPerSecond = lastVx + (vxMetersPerSecond - lastVx) * crossingT;
        double tofSec = tSec + NUMERICAL_DT_SEC * crossingT;
        Translation2d fieldContact = getFerryFieldPosition(distanceMeters, xAtContactMeters);
        double terrainHeightMeters = FieldTerrain.getSurfaceHeightMeters(fieldContact);
        boolean firstContactOnBump = FieldTerrain.isBumpFootprint(fieldContact);
        double marginMeters =
            parameters.entryRadiusMeters() - Math.abs(xAtContactMeters - distanceMeters);
        return new Result(
            marginMeters >= 0.0 && !firstContactOnBump,
            tofSec,
            marginMeters,
            Math.abs(vxAtContactMetersPerSecond) * FIELD_COEFFICIENT_OF_RESTITUTION,
            maxHeightMeters,
            fieldContact.getX(),
            fieldContact.getY(),
            terrainHeightMeters,
            firstContactOnBump);
      }

      lastClearanceMeters = clearanceMeters;
    }

    return Result.missed(
        parameters.maxFlightTimeSec(),
        -Double.MAX_VALUE,
        Double.POSITIVE_INFINITY,
        MISSED_MAX_HEIGHT_METERS);
  }

  public record Result(
      boolean scored,
      double tofSec,
      double marginMeters,
      double postBounceHorizontalVelocityMps,
      double maxHeightMeters,
      double firstContactXFieldMeters,
      double firstContactYFieldMeters,
      double firstContactSurfaceHeightMeters,
      boolean firstContactOnBump) {
    static Result missed(
        double tofSec,
        double marginMeters,
        double postBounceHorizontalVelocityMps,
        double maxHeightMeters) {
      return new Result(
          false,
          tofSec,
          marginMeters,
          postBounceHorizontalVelocityMps,
          maxHeightMeters,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          false);
    }
  }

  private double getFerryTerrainSurfaceHeightMeters(
      double distanceMeters, double pathDistanceMeters) {
    return FieldTerrain.getSurfaceHeightMeters(
        getFerryFieldPosition(distanceMeters, pathDistanceMeters));
  }

  private Translation2d getFerryFieldPosition(double distanceMeters, double pathDistanceMeters) {
    return new Translation2d(
        BLUE_LEFT_FERRY_TARGET_X_METERS + distanceMeters - pathDistanceMeters,
        BLUE_LEFT_FERRY_TARGET_Y_METERS);
  }

  private double getAnalyticalMaxHeightMeters(
      double verticalVelocityMetersPerSecond, double launchHeightMeters) {
    double gravityMetersPerSecondSquared = parameters.gravityMetersPerSecondSquared();
    if (gravityMetersPerSecondSquared <= 0.0) {
      return launchHeightMeters;
    }
    return launchHeightMeters
        + verticalVelocityMetersPerSecond
            * verticalVelocityMetersPerSecond
            / (2.0 * gravityMetersPerSecondSquared);
  }

  public record Parameters(
      double launchHeightMeters,
      double entryHeightMeters,
      double entryRadiusMeters,
      double gravityMetersPerSecondSquared,
      double maxFlightTimeSec,
      double effectiveFlywheelRadiusMeters,
      double exitVelocityScale,
      boolean simulateAirResistance,
      double hoodErrorDeg,
      double flywheelScaleError,
      double distanceErrorMeters,
      double launchAngleOffsetDeg) {
    public static Parameters rebuiltHub() {
      return new Parameters(
          TURRET_LAUNCH_HEIGHT_METERS,
          1.83,
          0.56,
          9.81,
          3.0,
          EFFECTIVE_FLYWHEEL_RADIUS_METERS,
          SHOTMAP_EXIT_VELOCITY_SCALE,
          true,
          0.4,
          0.015,
          0.08,
          SHOTMAP_LAUNCH_ANGLE_OFFSET_DEG);
    }
  }
}
