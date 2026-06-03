package frc.robot.shotmap;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.util.ShotTable2d;
import frc.robot.util.ShotTableTarget;
import java.util.List;
import org.junit.jupiter.api.Test;

class ShotMapGeneratorTest {
  @Test
  void previewGenerationIsDeterministicAndFillsEveryCell() {
    ShotMapConfig config =
        ShotMapConfig.preview()
            .withDistanceRange(2.0, 2.5, 0.5)
            .withRadialVelocityRange(-0.5, 0.5, 0.5)
            .withCoarseSearch(15.0, 20.0, 2.5, 30.0, 45.0, 5.0)
            .withRefinement(1.0, 2.0, 0.5, 1.0);
    ShotMapGenerator generator = new ShotMapGenerator(config);

    ShotMapGenerator.Output first = generator.generate();
    ShotMapGenerator.Output second = generator.generate();

    assertEquals(first.table(), second.table());
    assertEquals(List.of(2.0, 2.5), first.table().distanceMetersList());
    assertEquals(List.of(-0.5, 0.0, 0.5), first.table().radialVelocityMetersPerSecondList());

    for (ShotTable2d.Sample[] row : first.table().samples()) {
      for (ShotTable2d.Sample sample : row) {
        assertTrue(Double.isFinite(sample.hoodDeg()));
        assertTrue(Double.isFinite(sample.flywheelRps()));
        assertTrue(Double.isFinite(sample.tofSec()));
        assertFalse(Double.isNaN(sample.robustnessMargin()));
      }
    }
  }

  @Test
  void ferryGenerationProducesBounceValidatedTable() {
    ShotMapConfig config =
        ShotMapConfig.preview()
            .withTarget(ShotTableTarget.FERRY)
            .withDistanceRange(3.0, 3.0, 0.5)
            .withRadialVelocityRange(0.0, 0.0, 0.5)
            .withCoarseSearch(15.0, 35.0, 2.0, 25.0, 70.0, 5.0)
            .withRefinement(1.0, 3.0, 0.5, 1.0);

    ShotMapGenerator.Output output = new ShotMapGenerator(config).generate();
    ShotTable2d.Sample sample = output.table().samples()[0][0];

    assertTrue(sample.robustnessMargin() >= 0.0);
    assertTrue(sample.tofSec() < 1.5);
    assertTrue(output.diagnostics().get(0).maxHeightMeters() <= 2.2);
    assertTrue(output.diagnostics().get(0).postBounceHorizontalVelocityMetersPerSecond() > 0.0);
    assertEquals("FERRY", output.table().toFileFormat("preview", ShotTableTarget.FERRY).target);
  }

  @Test
  void ferryMovingAwayUsesMoreFlywheelToPreserveLandingTarget() {
    ShotMapConfig config =
        ShotMapConfig.preview()
            .withTarget(ShotTableTarget.FERRY)
            .withDistanceRange(3.75, 3.75, 0.5)
            .withRadialVelocityRange(-1.5, 0.0, 1.5)
            .withCoarseSearch(15.0, 35.0, 2.0, 25.0, 80.0, 5.0)
            .withRefinement(1.0, 3.0, 0.5, 1.0);

    ShotMapGenerator.Output output = new ShotMapGenerator(config).generate();
    ShotTable2d.Sample movingAway = output.table().samples()[0][0];
    ShotTable2d.Sample stationary = output.table().samples()[0][1];

    assertTrue(movingAway.robustnessMargin() >= 0.0);
    assertTrue(stationary.robustnessMargin() >= 0.0);
    assertTrue(movingAway.flywheelRps() > stationary.flywheelRps());
  }

  @Test
  void ferryFinalConfigCoversFullNeutralZoneDistanceRange() {
    ShotMapConfig config =
        ShotMapGeneratorCli.createConfigForTest("final", ShotTableTarget.FERRY, 1.0);

    assertEquals(3.0, config.minDistanceMeters());
    assertEquals(9.5, config.maxDistanceMeters());
    assertEquals(-4.5, config.minRadialVelocityMetersPerSecond());
    assertEquals(4.5, config.maxRadialVelocityMetersPerSecond());
  }

  @Test
  void ferryTerrainRejectsUnderpoweredShotThatContactsBumpBeforeTarget() {
    HubShotPhysics physics = new HubShotPhysics();

    HubShotPhysics.Result terrain = null;
    for (double distanceMeters = 6.0;
        distanceMeters <= 9.5 && terrain == null;
        distanceMeters += 0.25) {
      for (double hoodDeg = 13.0; hoodDeg <= 29.85 && terrain == null; hoodDeg += 0.5) {
        for (double flywheelRps = 30.0;
            flywheelRps <= 70.0 && terrain == null;
            flywheelRps += 0.5) {
          HubShotPhysics.Result candidateTerrain =
              physics.evaluateFerryFirstTerrainContact(distanceMeters, 0.0, hoodDeg, flywheelRps);
          if (candidateTerrain.firstContactOnBump()) {
            terrain = candidateTerrain;
          }
        }
      }
    }

    assertTrue(terrain != null, "expected to find an underpowered shot that contacts the bump");
    assertTrue(terrain.firstContactOnBump());
    assertFalse(terrain.scored());
    assertTrue(terrain.firstContactXFieldMeters() > 4.0);
    assertTrue(terrain.firstContactXFieldMeters() < 5.25);
  }
}
