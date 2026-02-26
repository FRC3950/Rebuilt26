package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

public class ZonesTest {
  @Test
  void containsBottomLeftTrench() {
    assertTrue(Zones.TRENCH_ZONES.contains(new Translation2d(4.5, 0.5)));
    assertFalse(Zones.TRENCH_ZONES.contains(new Translation2d(6.0, 0.5)));
  }

  @Test
  void containsTopRightTrench() {
    assertTrue(Zones.TRENCH_ZONES.contains(new Translation2d(11.9, 7.2)));
    assertFalse(Zones.TRENCH_ZONES.contains(new Translation2d(10.5, 7.2)));
  }

  @Test
  void predictiveTrueWhenMovingIntoZone() {
    Zones.PredictiveRectZone zone = new Zones.PredictiveRectZone(4.0, 5.0, 0.0, 1.0);
    Translation2d start = new Translation2d(3.5, 0.5);
    ChassisSpeeds speeds = new ChassisSpeeds(2.0, 0.0, 0.0);

    assertTrue(zone.willContain(start, speeds, 0.4));
  }

  @Test
  void predictiveFalseWhenMovingAwayFromZone() {
    Zones.PredictiveRectZone zone = new Zones.PredictiveRectZone(4.0, 5.0, 0.0, 1.0);
    Translation2d start = new Translation2d(3.5, 0.5);
    ChassisSpeeds speeds = new ChassisSpeeds(-2.0, 0.0, 0.0);

    assertFalse(zone.willContain(start, speeds, 0.4));
  }

  @Test
  void predictiveTrueForDiagonalSegmentIntersection() {
    Zones.PredictiveRectZone zone = new Zones.PredictiveRectZone(4.0, 5.0, 1.0, 2.0);
    Translation2d start = new Translation2d(3.0, 0.0);
    ChassisSpeeds speeds = new ChassisSpeeds(3.0, 3.0, 0.0);

    assertTrue(zone.willContain(start, speeds, 0.5));
  }

  @Test
  void collectionReturnsActiveZone() {
    Translation2d inTopLeft = new Translation2d(4.5, 7.2);
    ChassisSpeeds still = new ChassisSpeeds();

    assertTrue(Zones.TRENCH_ZONES.firstWillContain(inTopLeft, still, 0.4).isPresent());
  }
}
