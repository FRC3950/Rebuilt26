package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.*;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.SubsystemConstants.Turret;
import java.io.File;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class PrecomputedShotMapTest {
  private static final File MODEL = new File("src/main/deploy/chuck_shot_model.json");
  private static final ObjectMapper JSON = new ObjectMapper();
  @TempDir Path temporary;

  @Test
  void matchesTypeScriptSharedFixturesIncludingIntermediatePoints() throws Exception {
    var map = new PrecomputedShotMap(MODEL);
    JsonNode fixtures =
        JSON.readTree(new File("tools/shot-lab/test/fixtures/radial-v2-evaluation.json"));
    assertTrue(fixtures.size() >= 20);
    for (var f : fixtures) {
      double[] result = map.evaluate(f.path("distance").asDouble(), f.path("radial").asDouble());
      for (int i = 0; i < 7; i++)
        assertEquals(f.path("values").get(i).asDouble(), result[i], 1e-10);
    }
    assertThrows(IllegalArgumentException.class, () -> map.evaluate(9, 0));
    assertThrows(IllegalArgumentException.class, () -> map.evaluate(3, Double.NaN));
  }

  @Test
  void bothOffsetsRotationAndTiltPreserveTheCompensatedThreeDimensionalVector() throws Exception {
    var map = new PrecomputedShotMap(MODEL);
    var root = JSON.readTree(MODEL);
    int checked = 0;
    var c = SimpleShotModel.defaults();
    for (var p : root.path("patches")) {
      double d = p.path("center").get(0).asDouble(), r = p.path("center").get(1).asDouble();
      if (d < 3 || Math.abs(r) > 2) continue;
      double[] q = map.evaluate(d, r);
      for (var offset : new Translation2d[] {Turret.robotToTurret1, Turret.robotToTurret2}) {
        double omega = .7, tangent = 1.2, roll = .02, pitch = -.015;
        var pose = new Pose2d(-d - offset.getX(), -offset.getY(), Rotation2d.kZero);
        var motion =
            new ChassisSpeeds(r + omega * offset.getY(), tangent - omega * offset.getX(), omega);
        var s =
            PrecomputedShotMap.solve(
                map, "", c, pose, motion, new Translation2d(), offset, roll, pitch);
        if (!s.parameters().isValid())
          continue; // Physical hood limits can reject compensated requests.
        assertEquals(d, s.distance(), 1e-10);
        assertEquals(r, s.radial(), 1e-10);
        assertEquals(tangent, s.tangential(), 1e-10);
        var out = s.parameters();
        double a = Math.toRadians(77 + (out.hoodAngleDeg() - 13) * (60.15 - 77) / (29.85 - 13));
        double v = out.flywheelSpeed() * 2 * Math.PI * .0508, yaw = out.turretAngle().getRadians();
        double x = v * Math.cos(a) * Math.cos(yaw),
            y = v * Math.cos(a) * Math.sin(yaw),
            z = v * Math.sin(a);
        double ry = Math.cos(roll) * y - Math.sin(roll) * z,
            rz = Math.sin(roll) * y + Math.cos(roll) * z;
        double rx = Math.cos(pitch) * x + Math.sin(pitch) * rz,
            zz = -Math.sin(pitch) * x + Math.cos(pitch) * rz;
        assertEquals(q[1] * Math.cos(Math.toRadians(q[0])), rx, 1e-9);
        assertEquals(0, ry + tangent, 1e-9); // Radial motion must not be subtracted a second time.
        assertEquals(q[1] * Math.sin(Math.toRadians(q[0])), zz, 1e-9);
        var redPose = new Pose2d(-pose.getX(), -pose.getY(), Rotation2d.fromDegrees(180));
        var red =
            PrecomputedShotMap.solve(
                map,
                "",
                c,
                redPose,
                new ChassisSpeeds(-motion.vxMetersPerSecond, -motion.vyMetersPerSecond, omega),
                new Translation2d(),
                offset,
                roll,
                pitch);
        assertTrue(red.parameters().isValid());
        assertEquals(out.flywheelSpeed(), red.parameters().flywheelSpeed(), 1e-9);
        assertEquals(out.hoodAngleDeg(), red.parameters().hoodAngleDeg(), 1e-9);
        assertEquals(
            out.turretAngle().getRadians(), red.parameters().turretAngle().getRadians(), 1e-9);
        checked++;
      }
      if (checked >= 40) break;
    }
    assertTrue(checked >= 12, "Need multiple independently fitted regions for both turrets");
  }

  @Test
  void rejectsMissingIncompatibleMalformedAndChangedPhysics() throws Exception {
    var c = SimpleShotModel.defaults();
    var map = new PrecomputedShotMap(MODEL);
    var invalid =
        PrecomputedShotMap.solve(
            null,
            "Missing model",
            c,
            new Pose2d(),
            new ChassisSpeeds(),
            new Translation2d(3, 0),
            new Translation2d(),
            0,
            0);
    assertFalse(invalid.parameters().isValid());
    assertEquals(0, invalid.parameters().flywheelSpeed());
    var changed =
        PrecomputedShotMap.solve(
            map,
            "",
            c.with(SimpleShotModel.Param.DRAG, .1),
            new Pose2d(),
            new ChassisSpeeds(),
            new Translation2d(3, 0),
            new Translation2d(),
            0,
            0);
    assertFalse(changed.parameters().isValid());
    assertTrue(changed.parameters().invalidReason().contains("regenerate"));
    ObjectNode root = (ObjectNode) JSON.readTree(MODEL);
    root.put("version", 1);
    File bad = temporary.resolve("bad.json").toFile();
    JSON.writeValue(bad, root);
    assertThrows(IllegalArgumentException.class, () -> new PrecomputedShotMap(bad));
    root = (ObjectNode) JSON.readTree(MODEL);
    ((ObjectNode) root.path("patches").get(0)).putArray("coefficients").addArray().add(1);
    JSON.writeValue(bad, root);
    assertThrows(IllegalArgumentException.class, () -> new PrecomputedShotMap(bad));
  }
}
