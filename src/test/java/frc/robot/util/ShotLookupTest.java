package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShotLookupTest {
  private static final double EPSILON = 1e-9;

  @Test
  void twoDimensionalTableReturnsExactGridPoint() {
    ShotTable2d table =
        new ShotTable2d(
            new double[] {2.0, 4.0},
            new double[] {-1.0, 1.0},
            new ShotTable2d.Sample[][] {
              {
                new ShotTable2d.Sample(10.0, 40.0, 0.8, 0.1),
                new ShotTable2d.Sample(12.0, 44.0, 0.9, 0.2)
              },
              {
                new ShotTable2d.Sample(20.0, 50.0, 1.0, 0.3),
                new ShotTable2d.Sample(22.0, 54.0, 1.1, 0.4)
              }
            });

    ShotLookup lookup = ShotLookup.fromShotTable2d(table);

    Distancer shot = lookup.getShot(4.0, 1.0);

    assertEquals(22.0, shot.hoodAngleDeg(), EPSILON);
    assertEquals(54.0, shot.flywheelRps(), EPSILON);
    assertEquals(1.1, shot.tofSec(), EPSILON);
  }

  @Test
  void twoDimensionalTableBilinearlyInterpolatesBetweenGridPoints() {
    ShotTable2d table =
        new ShotTable2d(
            new double[] {2.0, 4.0},
            new double[] {-1.0, 1.0},
            new ShotTable2d.Sample[][] {
              {
                new ShotTable2d.Sample(10.0, 40.0, 0.8, 0.1),
                new ShotTable2d.Sample(12.0, 44.0, 0.9, 0.2)
              },
              {
                new ShotTable2d.Sample(20.0, 50.0, 1.0, 0.3),
                new ShotTable2d.Sample(22.0, 54.0, 1.1, 0.4)
              }
            });

    ShotLookup lookup = ShotLookup.fromShotTable2d(table);

    Distancer shot = lookup.getShot(3.0, 0.0);

    assertEquals(16.0, shot.hoodAngleDeg(), EPSILON);
    assertEquals(47.0, shot.flywheelRps(), EPSILON);
    assertEquals(0.95, shot.tofSec(), EPSILON);
  }

  @Test
  void twoDimensionalTableCanClampOutsideGrid() {
    ShotTable2d table =
        new ShotTable2d(
            new double[] {2.0, 4.0},
            new double[] {0.0, 2.0},
            new ShotTable2d.Sample[][] {
              {
                new ShotTable2d.Sample(10.0, 40.0, 0.8, 0.1),
                new ShotTable2d.Sample(12.0, 44.0, 0.9, 0.2)
              },
              {
                new ShotTable2d.Sample(20.0, 50.0, 1.0, 0.3),
                new ShotTable2d.Sample(22.0, 54.0, 1.1, 0.4)
              }
            });

    ShotLookup lookup = ShotLookup.fromShotTable2d(table);

    Distancer shot = lookup.getShotClamped(8.0, -3.0);

    assertEquals(20.0, shot.hoodAngleDeg(), EPSILON);
    assertEquals(50.0, shot.flywheelRps(), EPSILON);
    assertEquals(1.0, shot.tofSec(), EPSILON);
  }

  @Test
  void legacyDistanceOnlyLookupStillInterpolates() {
    Distancer.Row near = new Distancer.Row();
    near.d = 2.0;
    near.hoodDeg = 10.0;
    near.rps = 40.0;
    near.tof = 0.8;
    Distancer.Row far = new Distancer.Row();
    far.d = 4.0;
    far.hoodDeg = 20.0;
    far.rps = 50.0;
    far.tof = 1.0;

    ShotLookup lookup = ShotLookup.fromLegacyRows(java.util.List.of(near, far));

    Distancer shot = lookup.getShot(3.0, 4.5);

    assertEquals(15.0, shot.hoodAngleDeg(), EPSILON);
    assertEquals(45.0, shot.flywheelRps(), EPSILON);
    assertEquals(0.9, shot.tofSec(), EPSILON);
  }

  @Test
  void shotTableTargetsUseSeparateDeployFilenames() {
    assertEquals("shot_table_2d.json", ShotTableTarget.HUB.deployFilename());
    assertEquals("ferry_shot_table_2d.json", ShotTableTarget.FERRY.deployFilename());
    assertEquals("HUB", ShotTableTarget.HUB.fileTargetName());
    assertEquals("FERRY", ShotTableTarget.FERRY.fileTargetName());
  }

  @Test
  void fileFormatCarriesRequestedTargetName() {
    ShotTable2d table =
        new ShotTable2d(
            new double[] {2.0},
            new double[] {0.0},
            new ShotTable2d.Sample[][] {{new ShotTable2d.Sample(20.0, 50.0, 0.8, 0.1)}});

    ShotTable2d.FileFormat file = table.toFileFormat("preview", ShotTableTarget.FERRY);

    assertEquals("FERRY", file.target);
    assertTrue(file.points.get(0).get(0).tofSec() > 0.0);
  }
}
