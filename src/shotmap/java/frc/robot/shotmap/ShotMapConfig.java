package frc.robot.shotmap;

import static frc.robot.Constants.SubsystemConstants.Turret.maxFlywheelRps;
import static frc.robot.Constants.SubsystemConstants.Turret.maxHoodAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.minHoodAngle;

import frc.robot.util.ShotTableTarget;

public record ShotMapConfig(
    ShotTableTarget target,
    String mode,
    double minDistanceMeters,
    double maxDistanceMeters,
    double distanceStepMeters,
    double minRadialVelocityMetersPerSecond,
    double maxRadialVelocityMetersPerSecond,
    double radialVelocityStepMetersPerSecond,
    SearchRange coarseSearch,
    Refinement refinement,
    HubShotPhysics.Parameters physicsParameters) {
  public static ShotMapConfig preview() {
    return new ShotMapConfig(
        ShotTableTarget.HUB,
        "preview",
        1.0,
        7.0,
        0.5,
        -4.5,
        4.5,
        1.5,
        new SearchRange(minHoodAngle, maxHoodAngle, 1.0, 30.0, Math.min(maxFlywheelRps, 90.0), 2.0),
        new Refinement(1.0, 3.0, 0.25, 0.5),
        HubShotPhysics.Parameters.rebuiltHub());
  }

  public static ShotMapConfig finall() {
    return preview()
        .withMode("final")
        .withDistanceRange(1.0, 7.0, 0.25)
        .withRadialVelocityRange(-4.5, 4.5, 0.75)
        .withCoarseSearch(
            minHoodAngle, maxHoodAngle, 0.5, 30.0, Math.min(maxFlywheelRps, 95.0), 1.0)
        .withRefinement(0.75, 2.0, 0.1, 0.25);
  }

  public ShotMapConfig withMode(String mode) {
    return new ShotMapConfig(
        target,
        mode,
        minDistanceMeters,
        maxDistanceMeters,
        distanceStepMeters,
        minRadialVelocityMetersPerSecond,
        maxRadialVelocityMetersPerSecond,
        radialVelocityStepMetersPerSecond,
        coarseSearch,
        refinement,
        physicsParameters);
  }

  public ShotMapConfig withDistanceRange(double min, double max, double step) {
    return new ShotMapConfig(
        target,
        mode,
        min,
        max,
        step,
        minRadialVelocityMetersPerSecond,
        maxRadialVelocityMetersPerSecond,
        radialVelocityStepMetersPerSecond,
        coarseSearch,
        refinement,
        physicsParameters);
  }

  public ShotMapConfig withRadialVelocityRange(double min, double max, double step) {
    return new ShotMapConfig(
        target,
        mode,
        minDistanceMeters,
        maxDistanceMeters,
        distanceStepMeters,
        min,
        max,
        step,
        coarseSearch,
        refinement,
        physicsParameters);
  }

  public ShotMapConfig withCoarseSearch(
      double minHoodDeg,
      double maxHoodDeg,
      double hoodStepDeg,
      double minFlywheelRps,
      double maxFlywheelRps,
      double flywheelStepRps) {
    return new ShotMapConfig(
        target,
        mode,
        minDistanceMeters,
        maxDistanceMeters,
        distanceStepMeters,
        minRadialVelocityMetersPerSecond,
        maxRadialVelocityMetersPerSecond,
        radialVelocityStepMetersPerSecond,
        new SearchRange(
            minHoodDeg, maxHoodDeg, hoodStepDeg, minFlywheelRps, maxFlywheelRps, flywheelStepRps),
        refinement,
        physicsParameters);
  }

  public ShotMapConfig withRefinement(
      double hoodWindowDeg, double flywheelWindowRps, double hoodStepDeg, double flywheelStepRps) {
    return new ShotMapConfig(
        target,
        mode,
        minDistanceMeters,
        maxDistanceMeters,
        distanceStepMeters,
        minRadialVelocityMetersPerSecond,
        maxRadialVelocityMetersPerSecond,
        radialVelocityStepMetersPerSecond,
        coarseSearch,
        new Refinement(hoodWindowDeg, flywheelWindowRps, hoodStepDeg, flywheelStepRps),
        physicsParameters);
  }

  public ShotMapConfig withFlywheelScale(double flywheelScale) {
    HubShotPhysics.Parameters p = physicsParameters;
    HubShotPhysics.Parameters scaled =
        new HubShotPhysics.Parameters(
            p.launchHeightMeters(),
            p.entryHeightMeters(),
            p.entryRadiusMeters(),
            p.gravityMetersPerSecondSquared(),
            p.maxFlightTimeSec(),
            p.effectiveFlywheelRadiusMeters(),
            p.exitVelocityScale() * flywheelScale,
            p.simulateAirResistance(),
            p.hoodErrorDeg(),
            p.flywheelScaleError(),
            p.distanceErrorMeters(),
            p.launchAngleOffsetDeg());
    return new ShotMapConfig(
        target,
        mode,
        minDistanceMeters,
        maxDistanceMeters,
        distanceStepMeters,
        minRadialVelocityMetersPerSecond,
        maxRadialVelocityMetersPerSecond,
        radialVelocityStepMetersPerSecond,
        coarseSearch,
        refinement,
        scaled);
  }

  public ShotMapConfig withTarget(ShotTableTarget target) {
    return new ShotMapConfig(
        target,
        mode,
        minDistanceMeters,
        maxDistanceMeters,
        distanceStepMeters,
        minRadialVelocityMetersPerSecond,
        maxRadialVelocityMetersPerSecond,
        radialVelocityStepMetersPerSecond,
        coarseSearch,
        refinement,
        physicsParameters);
  }

  public record SearchRange(
      double minHoodDeg,
      double maxHoodDeg,
      double hoodStepDeg,
      double minFlywheelRps,
      double maxFlywheelRps,
      double flywheelStepRps) {}

  public record Refinement(
      double hoodWindowDeg, double flywheelWindowRps, double hoodStepDeg, double flywheelStepRps) {}
}
