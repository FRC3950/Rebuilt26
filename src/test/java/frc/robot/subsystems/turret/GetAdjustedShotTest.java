package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.Distancer;
import frc.robot.util.ShotLookup;
import frc.robot.util.ShotTableTarget;
import java.util.List;
import org.junit.jupiter.api.Test;

class GetAdjustedShotTest {
  private static final double EPSILON = 1e-9;

  @Test
  void usesFerryLookupWhenRequested() {
    Distancer.Row hub = row(2.0, 15.0, 45.0, 0.8);
    Distancer.Row ferry = row(2.0, 27.0, 34.0, 0.6);
    GetAdjustedShot shot =
        new GetAdjustedShot(
            ShotLookup.fromLegacyRows(List.of(hub)), ShotLookup.fromLegacyRows(List.of(ferry)));

    GetAdjustedShot.ShootingParameters params =
        shot.getParameters(
            Pose2d.kZero,
            new ChassisSpeeds(),
            new Translation2d(2.0, 0.0),
            Translation2d.kZero,
            0.0,
            ShotTableTarget.FERRY);

    assertEquals(27.0, params.hoodAngleDeg(), EPSILON);
    assertEquals(34.0, params.flywheelSpeed(), EPSILON);
  }

  @Test
  void defaultsToHubLookup() {
    Distancer.Row hub = row(2.0, 15.0, 45.0, 0.8);
    Distancer.Row ferry = row(2.0, 27.0, 34.0, 0.6);
    GetAdjustedShot shot =
        new GetAdjustedShot(
            ShotLookup.fromLegacyRows(List.of(hub)), ShotLookup.fromLegacyRows(List.of(ferry)));

    GetAdjustedShot.ShootingParameters params =
        shot.getParameters(
            Pose2d.kZero,
            new ChassisSpeeds(),
            new Translation2d(2.0, 0.0),
            Translation2d.kZero,
            0.0);

    assertEquals(15.0, params.hoodAngleDeg(), EPSILON);
    assertEquals(45.0, params.flywheelSpeed(), EPSILON);
  }

  private static Distancer.Row row(double distance, double hoodDeg, double rps, double tof) {
    Distancer.Row row = new Distancer.Row();
    row.d = distance;
    row.hoodDeg = hoodDeg;
    row.rps = rps;
    row.tof = tof;
    return row;
  }
}
