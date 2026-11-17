package frc.robot.subsystems.turret.demo;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DemoConstants.LimelightRobotSpaceAxes;
import frc.robot.util.Distancer;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class BallisticShotModelTest {
  private static final double G = BallisticShotModel.GRAVITY;
  // Simple known model: launch angle = 80 - hood, exit speed = 0.15 * rps.
  private static final BallisticShotModel MODEL = new BallisticShotModel(80.0, -1.0, 0.0, 0.15);

  /** Height of a drag-free ball at horizontal distance x. */
  private static double heightAt(BallisticShotModel model, double hood, double rps, double x) {
    double theta = Math.toRadians(model.launchAngleDegForHood(hood));
    double v = model.exitSpeedForRps(rps);
    double t = x / (v * Math.cos(theta));
    return v * Math.sin(theta) * t - 0.5 * G * t * t;
  }

  @Test
  void solutionPassesThroughTarget() {
    double[][] targets = {{3.0, 1.0}, {6.0, 2.0}, {8.0, 0.5}, {2.0, -0.2}};
    for (double[] target : targets) {
      var s = MODEL.solve(target[0], target[1], 13.0, 29.85, 100.0, 0.0, Double.NaN, 0.25);
      assertTrue(s.valid(), s.reason());
      assertEquals(target[1], heightAt(MODEL, s.hoodDeg(), s.flywheelRps(), target[0]), 1e-6);
    }
  }

  @Test
  void hoodOverrideIsHonored() {
    var s = MODEL.solve(5.0, 1.0, 13.0, 29.85, 100.0, 20.0, Double.NaN, 0.25);
    assertTrue(s.valid());
    assertEquals(20.0, s.hoodDeg(), 1e-9);
    assertEquals(1.0, heightAt(MODEL, 20.0, s.flywheelRps(), 5.0), 1e-6);
  }

  @Test
  void preferredHoodIsUsedWhenReachableElseFallsBack() {
    var preferred = MODEL.solve(5.0, 1.0, 13.0, 29.85, 100.0, 0.0, 15.0, 0.25);
    assertTrue(preferred.valid());
    assertEquals(15.0, preferred.hoodDeg(), 1e-9);
    // Too little flywheel headroom for the steep preferred arc -> search a flatter hood.
    var capped = MODEL.solve(5.0, 1.0, 13.0, 29.85, preferred.flywheelRps() - 1.0, 0.0, 15.0, 0.25);
    assertTrue(capped.valid(), capped.reason());
    assertTrue(capped.hoodDeg() > 15.0);
  }

  @Test
  void higherTargetNeedsMoreSpeed() {
    var low = MODEL.solve(5.0, 0.5, 20.0, 20.0, 100.0, 20.0, Double.NaN, 0.25);
    var high = MODEL.solve(5.0, 2.5, 20.0, 20.0, 100.0, 20.0, Double.NaN, 0.25);
    assertTrue(high.flywheelRps() > low.flywheelRps());
  }

  @Test
  void unreachableTargetIsInvalid() {
    assertFalse(MODEL.solve(30.0, 2.0, 13.0, 29.85, 100.0, 0.0, Double.NaN, 0.25).valid());
    assertFalse(MODEL.solve(0.0, 2.0, 13.0, 29.85, 100.0, 0.0, Double.NaN, 0.25).valid());
  }

  @Test
  void uncalibratedModelRefusesToShoot() {
    var model = BallisticShotModel.fitFromShotTable(List.of(), 0.45, 1.83, 0.53);
    assertFalse(model.isCalibrated());
    assertFalse(model.solve(4.0, 1.0, 13.0, 29.85, 100.0, 0.0, Double.NaN, 0.25).valid());
  }

  @Test
  void fitRecoversModelFromSyntheticHubTable() {
    double launchHeight = 0.45;
    double impactHeight = 1.83;
    double beyond = 0.53;
    List<Distancer.Row> rows = new ArrayList<>();
    double[][] hoodRps = {{13, 40}, {16, 48}, {20, 55}, {24, 62}, {28, 70}};
    for (double[] hr : hoodRps) {
      double theta = Math.toRadians(MODEL.launchAngleDegForHood(hr[0]));
      double v = MODEL.exitSpeedForRps(hr[1]);
      double vx = v * Math.cos(theta);
      double vz = v * Math.sin(theta);
      double dz = impactHeight - launchHeight;
      // Descending-branch time to reach impact height.
      double t = (vz + Math.sqrt(vz * vz - 2 * G * dz)) / G;
      Distancer.Row row = new Distancer.Row();
      row.d = vx * t - beyond;
      row.hoodDeg = hr[0];
      row.rps = hr[1];
      row.tof = t;
      rows.add(row);
    }
    var fit = BallisticShotModel.fitFromShotTable(rows, launchHeight, impactHeight, beyond);
    assertTrue(fit.isCalibrated());
    assertEquals(MODEL.launchAngleDegForHood(18.0), fit.launchAngleDegForHood(18.0), 1e-6);
    assertEquals(MODEL.exitSpeedForRps(50.0), fit.exitSpeedForRps(50.0), 1e-6);
  }

  @Test
  void realShotTableGivesSaneModel() {
    // Uses src/main/deploy/shot_table.json via the deploy directory.
    var rows = Distancer.loadRowsFromDeploy("shot_table.json");
    if (rows.isEmpty()) {
      return;
    }
    var model = BallisticShotModel.fitFromShotTable(rows, 0.445, 1.8288, 0.5296);
    assertTrue(model.isCalibrated());
    // Higher hood = flatter shot; more rps = faster ball.
    assertTrue(model.launchAngleDegForHood(13.0) > model.launchAngleDegForHood(29.0));
    assertTrue(model.exitSpeedForRps(60.0) > model.exitSpeedForRps(40.0));
  }

  @Test
  void limelightAxesConvertToWpilibRobotFrame() {
    var flu =
        DemoTargetTracker.toRobotFrame(
            new double[] {2.0, 0.5, 1.5, 0, 0, 0}, LimelightRobotSpaceAxes.FORWARD_LEFT_UP);
    assertEquals(new Translation3d(2.0, 0.5, 1.5), flu);
    // Camera-style: x right, y down, z forward -> 2 m ahead, 0.5 m left, 1.5 m up.
    var rdf =
        DemoTargetTracker.toRobotFrame(
            new double[] {-0.5, -1.5, 2.0, 0, 0, 0}, LimelightRobotSpaceAxes.RIGHT_DOWN_FORWARD);
    assertEquals(new Translation3d(2.0, 0.5, 1.5), rdf);
  }

  @Test
  void targetMovesOppositeToRobotMotion() {
    Translation3d tag = new Translation3d(3.0, 0.0, 1.2);
    // Drive forward 1 m/s for 0.5 s: tag gets 0.5 m closer.
    Translation3d drove =
        DemoTargetTracker.propagate(tag, new ChassisSpeeds(1.0, 0.0, 0.0), Rotation2d.kZero, 0.5);
    assertEquals(2.5, drove.getX(), 1e-9);
    assertEquals(0.0, drove.getY(), 1e-9);
    assertEquals(1.2, drove.getZ(), 1e-9);
    // Turn 90 deg left in place: tag that was ahead is now on the right.
    Translation3d turned =
        DemoTargetTracker.propagate(tag, new ChassisSpeeds(), Rotation2d.kCCW_90deg, 0.02);
    assertEquals(0.0, turned.getX(), 1e-9);
    assertEquals(-3.0, turned.getY(), 1e-9);
  }
}
