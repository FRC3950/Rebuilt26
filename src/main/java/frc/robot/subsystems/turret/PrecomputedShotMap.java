package frc.robot.subsystems.turret;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.SubsystemConstants.Turret;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/** Radial quadratic lookup. Trajectory searches and fitting run exclusively on the laptop. */
public final class PrecomputedShotMap {
  private record Patch(double[] bounds, double[] center, double[] scale, double[][] coefficients) {}

  private final List<Patch> patches = new ArrayList<>();
  private final double[] configuration;
  private static final SimpleShotModel.Param[] MATCH = {
    SimpleShotModel.Param.HEIGHT,
    SimpleShotModel.Param.RADIUS,
    SimpleShotModel.Param.ELEV_MIN,
    SimpleShotModel.Param.ELEV_MAX,
    SimpleShotModel.Param.TRANSFER,
    SimpleShotModel.Param.DRAG,
    SimpleShotModel.Param.SPIN_RATIO,
    SimpleShotModel.Param.LIFT,
    SimpleShotModel.Param.SPIN_DECAY
  };
  private static final String[] KEYS = {
    "height", "radius", "angleMin", "angleMax", "transfer", "drag", "spinRatio", "lift", "spinDecay"
  };

  private static final class Loaded {
    static final PrecomputedShotMap MAP;
    static final String ERROR;

    static {
      PrecomputedShotMap map = null;
      String error = "";
      try {
        map =
            new PrecomputedShotMap(
                new File(Filesystem.getDeployDirectory(), "chuck_shot_model.json"));
      } catch (IOException | IllegalArgumentException e) {
        error = "Invalid precomputed model: " + e.getMessage();
      }
      MAP = map;
      ERROR = error;
    }
  }

  /** Called during Tune construction so file I/O never occurs on the first enabled shot. */
  static void initializeDashboard() {
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString(
        SimpleShotModel.ROOT + "Map Status",
        Loaded.MAP == null
            ? Loaded.ERROR
            : "Version 2 radial quadratic loaded; physical calibration unverified");
  }

  public PrecomputedShotMap(File file) throws IOException {
    JsonNode root = new ObjectMapper().readTree(file);
    require(
        root != null
            && root.path("version").asInt() == 2
            && root.path("kind").asText().equals("radial-quadratic"),
        "Incompatible model version");
    require(
        root.path("units").path("angle").asText().equals("deg")
            && root.path("units").path("speed").asText().equals("m/s")
            && root.path("units").path("radial").asText().equals("m/s")
            && root.path("units").path("distance").asText().equals("m")
            && root.path("units").path("tof").asText().equals("s"),
        "Incompatible coefficient units");
    String[] basis = {"1", "d", "r", "d*d", "d*r", "r*r"};
    require(root.path("basis").size() == 6, "Incompatible polynomial basis");
    for (int i = 0; i < 6; i++)
      require(root.path("basis").get(i).asText().equals(basis[i]), "Incompatible polynomial basis");
    configuration = new double[KEYS.length];
    for (int i = 0; i < KEYS.length; i++)
      configuration[i] = number(root.path("config").path(KEYS[i]));
    require(
        root.path("validation").path("physicalValidation").isBoolean(),
        "Missing validation status");
    require(
        number(root.path("validation").path("maxErrorFraction")) < .1,
        "Fit error exceeds margin budget");
    for (JsonNode p : root.path("patches")) {
      double[] bounds = array(p.path("bounds"), 4),
          center = array(p.path("center"), 2),
          scale = array(p.path("scale"), 2);
      require(
          bounds[1] > bounds[0] && bounds[3] > bounds[2] && scale[0] > 0 && scale[1] > 0,
          "Invalid patch domain");
      require(
          number(p.path("maxErrorFraction")) >= 0 && number(p.path("maxErrorFraction")) < .1,
          "Unvalidated patch");
      require(p.path("coefficients").size() == 7, "Expected seven quadratic outputs");
      require(
          Math.abs(center[0] - (bounds[0] + bounds[1]) / 2) < 1e-8
              && Math.abs(center[1] - (bounds[2] + bounds[3]) / 2) < 1e-8
              && Math.abs(scale[0] - (bounds[1] - bounds[0]) / 2) < 1e-8
              && Math.abs(scale[1] - (bounds[3] - bounds[2]) / 2) < 1e-8,
          "Inconsistent patch normalization");
      double[][] coefficients = new double[7][];
      for (int i = 0; i < 7; i++) coefficients[i] = array(p.path("coefficients").get(i), 6);
      patches.add(new Patch(bounds, center, scale, coefficients));
    }
    require(!patches.isEmpty(), "No validated patches");
  }

  private static void require(boolean b, String message) {
    if (!b) throw new IllegalArgumentException(message);
  }

  private static double number(JsonNode n) {
    require(
        n != null && n.isNumber() && Double.isFinite(n.asDouble()),
        "Missing or non-finite coefficient/configuration");
    return n.asDouble();
  }

  private static double[] array(JsonNode n, int count) {
    require(n.isArray() && n.size() == count, "Malformed coefficient array");
    double[] values = new double[count];
    for (int i = 0; i < count; i++) values[i] = number(n.get(i));
    return values;
  }

  public double[] evaluate(double distance, double radial) {
    require(Double.isFinite(distance) && Double.isFinite(radial), "Non-finite map input");
    for (Patch p : patches)
      if (distance >= p.bounds[0]
          && distance <= p.bounds[1]
          && radial >= p.bounds[2]
          && radial <= p.bounds[3]) {
        double d = (distance - p.center[0]) / p.scale[0], r = (radial - p.center[1]) / p.scale[1];
        double[] b = {1, d, r, d * d, d * r, r * r}, v = new double[7];
        for (int i = 0; i < 7; i++) for (int j = 0; j < 6; j++) v[i] += p.coefficients[i][j] * b[j];
        for (double x : v) require(Double.isFinite(x), "Non-finite fitted output");
        require(v[0] > 0 && v[0] < 90 && v[1] > 0 && v[2] > 0, "Invalid fitted launch");
        for (int i = 3; i < 7; i++) require(v[i] > 0, "Invalid fitted margin");
        return v;
      }
    throw new IllegalArgumentException(
        "Unvalidated map region: inspect coverage gaps / regenerate map");
  }

  public static SimpleShotModel.Solution solve(
      SimpleShotModel.Settings c,
      Pose2d pose,
      ChassisSpeeds velocity,
      Translation2d target,
      Translation2d offset,
      double roll,
      double pitch) {
    return solve(Loaded.MAP, Loaded.ERROR, c, pose, velocity, target, offset, roll, pitch);
  }

  public static SimpleShotModel.Solution solve(
      PrecomputedShotMap map,
      String error,
      SimpleShotModel.Settings c,
      Pose2d pose,
      ChassisSpeeds velocity,
      Translation2d target,
      Translation2d offset,
      double roll,
      double pitch) {
    double distance = 0, radial = 0, tangential = 0;
    try {
      require(map != null, error.isEmpty() ? "Missing precomputed model" : error);
      require(c.error().isEmpty(), c.error());
      for (double v :
          new double[] {
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians(),
            velocity.vxMetersPerSecond,
            velocity.vyMetersPerSecond,
            velocity.omegaRadiansPerSecond,
            roll,
            pitch,
            target.getX(),
            target.getY()
          }) require(Double.isFinite(v), "Non-finite robot state / tilt unavailable");
      for (int i = 0; i < MATCH.length; i++)
        require(
            Math.abs(c.get(MATCH[i]) - map.configuration[i]) < 1e-8,
            "Physics changed: regenerate map (" + KEYS[i] + ")");
      double delay = c.get(SimpleShotModel.Param.DELAY),
          heading = pose.getRotation().getRadians() + velocity.omegaRadiansPerSecond * delay;
      Translation2d o = offset.rotateBy(Rotation2d.fromRadians(heading));
      double dx = target.getX() - pose.getX() - velocity.vxMetersPerSecond * delay - o.getX(),
          dy = target.getY() - pose.getY() - velocity.vyMetersPerSecond * delay - o.getY();
      distance = Math.hypot(dx, dy);
      require(distance > 1e-6, "Turret coincides with target");
      double bearing = Math.atan2(dy, dx),
          vx = velocity.vxMetersPerSecond - velocity.omegaRadiansPerSecond * o.getY(),
          vy = velocity.vyMetersPerSecond + velocity.omegaRadiansPerSecond * o.getX();
      radial = (vx * dx + vy * dy) / distance;
      tangential = (-vx * dy + vy * dx) / distance;
      require(Math.hypot(radial, tangential) <= 4.25, "Turret velocity outside tested envelope");
      double[] q = map.evaluate(distance, radial);
      // Radial motion is already in the fit. Subtract ONLY tangential velocity in 3D.
      double a = Math.toRadians(q[0]),
          x = q[1] * Math.cos(a),
          y = -tangential,
          z = q[1] * Math.sin(a);
      double speed = Math.hypot(Math.hypot(x, y), z), yaw = bearing - heading;
      double xx = x * Math.cos(yaw) - y * Math.sin(yaw), yy = x * Math.sin(yaw) + y * Math.cos(yaw);
      double tx = Math.cos(pitch) * xx - Math.sin(pitch) * z,
          tz = Math.sin(pitch) * xx + Math.cos(pitch) * z;
      double ty = Math.cos(roll) * yy + Math.sin(roll) * tz,
          zz = -Math.sin(roll) * yy + Math.cos(roll) * tz;
      double elevation = Math.toDegrees(Math.atan2(zz, Math.hypot(tx, ty)));
      boolean left = offset.getY() >= 0;
      double hood =
          SimpleShotModel.hoodForElevation(c, elevation)
              + c.get(left ? SimpleShotModel.Param.LEFT_HOOD : SimpleShotModel.Param.RIGHT_HOOD);
      double rps = SimpleShotModel.wheelRps(c, speed, left);
      require(
          Double.isFinite(hood)
              && Double.isFinite(rps)
              && hood >= Turret.minHoodAngle
              && hood <= Turret.maxHoodAngle,
          "Compensated hood outside mechanism limits");
      require(
          rps >= 0 && rps <= Turret.maxFlywheelRps,
          "Compensated wheel speed outside mechanism limits");
      return new SimpleShotModel.Solution(
          new GetAdjustedShot.ShootingParameters(
              true,
              Rotation2d.fromRadians(
                  Math.atan2(ty, tx)
                      + Math.toRadians(
                          c.get(
                              left
                                  ? SimpleShotModel.Param.LEFT_YAW
                                  : SimpleShotModel.Param.RIGHT_YAW))),
              0,
              hood,
              rps,
              ""),
          distance,
          radial,
          tangential,
          q[2]);
    } catch (IllegalArgumentException e) {
      return new SimpleShotModel.Solution(
          new GetAdjustedShot.ShootingParameters(false, Rotation2d.kZero, 0, 0, 0, e.getMessage()),
          distance,
          radial,
          tangential,
          0);
    }
  }
}
