package frc.robot.sim.shottableanalysis;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.sim.FuelLaunchCalculator;
import frc.robot.sim.FuelSim;
import frc.robot.subsystems.turret.GetAdjustedShot;
import frc.robot.util.Distancer;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.IntStream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfEnvironmentVariable;

/**
 * Throwaway shot-table analysis harness (see shot_table_analysis/README.md). It drives the real
 * FuelSim and the real FuelLaunchCalculator, but feeds the calculator a LaunchCalibration rebuilt
 * from frozen points so editing shot_table.json does not move the measuring stick.
 *
 * <p>Never runs in a normal build: it is skipped unless SHOT_TABLE_ANALYSIS is set to a mode name.
 */
@EnabledIfEnvironmentVariable(named = "SHOT_TABLE_ANALYSIS", matches = ".+")
class ShotTableAnalysisTest {
  private static final String MODE = System.getenv("SHOT_TABLE_ANALYSIS");
  private static final Path DIR = Path.of("shot_table_analysis");
  private static final Path FROZEN = DIR.resolve("frozen_baseline.json");
  private static final Path DEPLOY_TABLE = Path.of("src/main/deploy/shot_table.json");
  private static final ObjectMapper JSON = new ObjectMapper();

  // Perturbations used for the robustness count (assumed tolerances, not measured robot values).
  private static final double AIM_TOL_DEG = 1.0;
  private static final double HOOD_TOL_DEG = 1.0;
  private static final double RPS_TOL_FRACTION = 0.025;

  // Shootable region for the turret launch point (blue alliance zone, see README for why).
  private static final double REGION_X_MIN = 0.30;
  private static final double REGION_X_MAX = 3.75;
  private static final double REGION_MARGIN_Y = 0.30;

  private static final double SIM_PERIOD = 0.02;
  private static final int MAX_STEPS = 250;

  // ---------------------------------------------------------------- reflection into the sim

  private static final Class<?> CAL_CLASS =
      cls("frc.robot.sim.FuelLaunchCalculator$LaunchCalibration");
  private static final Class<?> POINT_CLASS =
      cls("frc.robot.sim.FuelLaunchCalculator$LaunchCalibration$CalibrationPoint");
  private static final Class<?> FUEL_CLASS = cls("frc.robot.sim.FuelSim$Fuel");

  static final Translation2d SIM_HUB_CENTER =
      (Translation2d) staticField(FuelSim.Hub.class, "BLUE_HUB", "center");
  static final double ENTRY_HEIGHT = (double) staticField(FuelSim.Hub.class, "ENTRY_HEIGHT", null);
  static final double ENTRY_RADIUS = (double) staticField(FuelSim.Hub.class, "ENTRY_RADIUS", null);
  static final double FIELD_WIDTH = (double) staticField(FuelSim.class, "FIELD_WIDTH", null);

  // What the robot code aims at and measures distance to (blue alliance, no flip).
  static final Translation2d AIM_POINT = Constants.FieldConstants.hubGuy;

  private static Class<?> cls(String name) {
    try {
      return Class.forName(name);
    } catch (ClassNotFoundException e) {
      throw new IllegalStateException(e);
    }
  }

  private static Object staticField(Class<?> owner, String name, String innerField) {
    try {
      Field f = owner.getDeclaredField(name);
      f.setAccessible(true);
      Object value = f.get(null);
      if (innerField == null) {
        return value;
      }
      Field inner = value.getClass().getDeclaredField(innerField);
      inner.setAccessible(true);
      return inner.get(value);
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException(e);
    }
  }

  /** The calibration the sim derives right now from src/main/deploy/shot_table.json. */
  static Object derivedCalibration() throws ReflectiveOperationException {
    Method m = CAL_CLASS.getDeclaredMethod("fromShotTable");
    m.setAccessible(true);
    return m.invoke(null);
  }

  static List<double[]> calibrationPoints(Object calibration, String field)
      throws ReflectiveOperationException {
    Field f = CAL_CLASS.getDeclaredField(field);
    f.setAccessible(true);
    Method input = POINT_CLASS.getDeclaredMethod("input");
    Method output = POINT_CLASS.getDeclaredMethod("output");
    input.setAccessible(true);
    output.setAccessible(true);
    List<double[]> out = new ArrayList<>();
    for (Object p : (List<?>) f.get(calibration)) {
      out.add(new double[] {(double) input.invoke(p), (double) output.invoke(p)});
    }
    return out;
  }

  static Object calibrationFromPoints(List<double[]> hood, List<double[]> rps)
      throws ReflectiveOperationException {
    Constructor<?> pointCtor = POINT_CLASS.getDeclaredConstructor(double.class, double.class);
    pointCtor.setAccessible(true);
    List<Object> hoodPoints = new ArrayList<>();
    for (double[] p : hood) {
      hoodPoints.add(pointCtor.newInstance(p[0], p[1]));
    }
    List<Object> rpsPoints = new ArrayList<>();
    for (double[] p : rps) {
      rpsPoints.add(pointCtor.newInstance(p[0], p[1]));
    }
    Constructor<?> calCtor = CAL_CLASS.getDeclaredConstructor(List.class, List.class);
    calCtor.setAccessible(true);
    return calCtor.newInstance(hoodPoints, rpsPoints);
  }

  static FuelLaunchCalculator calculatorFor(Object calibration)
      throws ReflectiveOperationException {
    Constructor<FuelLaunchCalculator> ctor =
        FuelLaunchCalculator.class.getDeclaredConstructor(CAL_CLASS);
    ctor.setAccessible(true);
    return ctor.newInstance(calibration);
  }

  static Object frozenCalibration() throws IOException, ReflectiveOperationException {
    var node = JSON.readTree(FROZEN.toFile());
    return calibrationFromPoints(
        readPairs(node.get("hoodDegToLaunchAngleDeg")),
        readPairs(node.get("rpsToExitSpeedMetersPerSecond")));
  }

  private static List<double[]> readPairs(com.fasterxml.jackson.databind.JsonNode array) {
    List<double[]> out = new ArrayList<>();
    for (var pair : array) {
      out.add(new double[] {pair.get(0).asDouble(), pair.get(1).asDouble()});
    }
    return out;
  }

  /** frozen (default) or derived (whatever the sim derives from the deploy table right now). */
  static FuelLaunchCalculator selectedCalculator() throws Exception {
    String cal = env("STA_CAL", "frozen");
    return calculatorFor(cal.equals("derived") ? derivedCalibration() : frozenCalibration());
  }

  // ---------------------------------------------------------------- shot table lookup

  /** Mirrors GetAdjustedShot.getShotForDistance for an arbitrary table file. */
  static final class Table {
    final List<Distancer.Row> rows;
    final InterpolatingTreeMap<Double, Distancer> map =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Distancer::interpolate);

    Table(Path path) throws IOException {
      Distancer.DistancerFile file = JSON.readValue(path.toFile(), Distancer.DistancerFile.class);
      rows = new ArrayList<>(file.points);
      rows.sort(Comparator.comparingDouble(r -> r.d));
      for (var r : rows) {
        map.put(r.d, new Distancer(r.hoodDeg, r.rps, r.tof));
      }
    }

    Distancer get(double d) {
      if (rows.size() == 1) {
        return Distancer.fromRow(rows.get(0));
      }
      if (d < rows.get(0).d) {
        return lerpRows(rows.get(0), rows.get(1), d);
      }
      if (d > rows.get(rows.size() - 1).d) {
        return lerpRows(rows.get(rows.size() - 2), rows.get(rows.size() - 1), d);
      }
      return map.get(d);
    }

    private static Distancer lerpRows(Distancer.Row a, Distancer.Row b, double d) {
      return Distancer.fromRow(a).interpolate(Distancer.fromRow(b), (d - a.d) / (b.d - a.d));
    }
  }

  // ---------------------------------------------------------------- single-shot simulation

  record ShotResult(
      boolean scored,
      double tof, // launch to crossing ENTRY_HEIGHT on the way down (s); NaN if never crossed
      double crossAlong, // + = long, along the line from launch point to sim hub center (m)
      double crossLateral, // signed lateral offset at the crossing (m)
      boolean bounced, // horizontal velocity reversed or dropped sharply before the crossing
      double apex) {
    double crossRadius() {
      return Math.hypot(crossAlong, crossLateral);
    }
  }

  /** One FuelSim per thread; only one fuel is ever on the field so shots are independent. */
  static final class Sim {
    private static final AtomicInteger IDS = new AtomicInteger();
    private final FuelSim fuelSim = new FuelSim("/ShotTableAnalysis/" + IDS.incrementAndGet());
    private final List<?> fuels;
    private final Field posField;
    private final Field velField;

    Sim() {
      fuelSim.enableAirResistance();
      try {
        Field f = FuelSim.class.getDeclaredField("fuels");
        f.setAccessible(true);
        fuels = (List<?>) f.get(fuelSim);
        posField = FUEL_CLASS.getDeclaredField("pos");
        velField = FUEL_CLASS.getDeclaredField("vel");
        posField.setAccessible(true);
        velField.setAccessible(true);
      } catch (ReflectiveOperationException e) {
        throw new IllegalStateException(e);
      }
    }

    private Translation3d get(Field f, Object fuel) {
      try {
        return (Translation3d) f.get(fuel);
      } catch (IllegalAccessException e) {
        throw new IllegalStateException(e);
      }
    }

    ShotResult shoot(
        FuelLaunchCalculator calc,
        Translation2d launch,
        double fieldYawDeg,
        double hoodDeg,
        double rps) {
      fuelSim.clearFuel();
      var state =
          calc.calculateTurretLaunch(
              new Pose2d(launch, Rotation2d.kZero),
              new ChassisSpeeds(),
              Translation2d.kZero,
              fieldYawDeg,
              hoodDeg,
              rps);
      fuelSim.spawnFuel(state.position(), state.velocity());
      Object fuel = fuels.get(0);

      Translation2d toHub = SIM_HUB_CENTER.minus(launch);
      Translation2d along = toHub.div(toHub.getNorm());
      Translation3d prevP = state.position();
      Translation3d prevV = state.velocity();
      double apex = prevP.getZ();
      boolean bounced = false;

      for (int step = 1; step <= MAX_STEPS; step++) {
        fuelSim.stepSim();
        Translation3d p = get(posField, fuel);
        Translation3d v = get(velField, fuel);

        boolean teleported = p.minus(prevP).getNorm() > 0.5; // FuelSim moves scored fuel to exit
        boolean crossedDown = prevP.getZ() > ENTRY_HEIGHT && p.getZ() <= ENTRY_HEIGHT;
        if (teleported || crossedDown) {
          double tau = crossingTime(prevP.getZ(), prevV.getZ());
          Translation2d cross =
              new Translation2d(
                      prevP.getX() + prevV.getX() * tau, prevP.getY() + prevV.getY() * tau)
                  .minus(SIM_HUB_CENTER);
          double a = cross.getX() * along.getX() + cross.getY() * along.getY();
          double l = -cross.getX() * along.getY() + cross.getY() * along.getX();
          return new ShotResult(
              teleported,
              (step - 1) * SIM_PERIOD + tau,
              a,
              l,
              bounced,
              Math.max(apex, prevP.getZ()));
        }

        Translation2d hPrev = prevV.toTranslation2d();
        Translation2d hNow = v.toTranslation2d();
        if (hPrev.getNorm() > 0.5
            && (hPrev.dot(hNow) < 0 || hNow.getNorm() < 0.8 * hPrev.getNorm())) {
          bounced = true;
        }
        apex = Math.max(apex, p.getZ());
        if (v.getZ() < 0 && p.getZ() < ENTRY_HEIGHT) {
          break; // already below the entry on the way down; can never score from here
        }
        prevP = p;
        prevV = v;
      }
      return new ShotResult(false, Double.NaN, Double.NaN, Double.NaN, bounced, apex);
    }

    /** Ballistic estimate of when, inside one 20 ms step, the fuel crosses the entry height. */
    private static double crossingTime(double z0, double vz) {
      double g = 9.81;
      double disc = vz * vz + 2 * g * (z0 - ENTRY_HEIGHT);
      double tau = disc < 0 ? 0 : (vz + Math.sqrt(disc)) / g;
      return Math.max(0, Math.min(SIM_PERIOD, tau));
    }
  }

  private static final ThreadLocal<Sim> SIM = ThreadLocal.withInitial(Sim::new);

  // ---------------------------------------------------------------- robot-equivalent shot

  record Setpoint(double yawDeg, double hoodDeg, double rps, double robotDistance) {}

  /** What the robot commands for a stationary turret at this launch point (stationary path). */
  static Setpoint robotSetpoint(Table table, Translation2d launch) {
    double d = AIM_POINT.getDistance(launch);
    Distancer shot = table.get(d);
    double hood =
        Math.max(
            Constants.SubsystemConstants.Turret.minHoodAngle,
            Math.min(Constants.SubsystemConstants.Turret.maxHoodAngle, shot.hoodAngleDeg()));
    double rps =
        Math.max(
            Constants.SubsystemConstants.Turret.minFlywheelRps,
            Math.min(Constants.SubsystemConstants.Turret.maxFlywheelRps, shot.flywheelRps()));
    double yaw = AIM_POINT.minus(launch).getAngle().getDegrees();
    return new Setpoint(yaw, hood, rps, d);
  }

  static final double[] PERTURB = {-1, 0, 1};

  /** Scores out of 27: every combination of {-tol, 0, +tol} on aim, hood and rps. */
  static int perturbedScore(FuelLaunchCalculator calc, Translation2d launch, Setpoint s) {
    int scored = 0;
    for (double a : PERTURB) {
      for (double h : PERTURB) {
        for (double r : PERTURB) {
          if (SIM.get()
              .shoot(
                  calc,
                  launch,
                  s.yawDeg() + a * AIM_TOL_DEG,
                  s.hoodDeg() + h * HOOD_TOL_DEG,
                  s.rps() * (1 + r * RPS_TOL_FRACTION))
              .scored()) {
            scored++;
          }
        }
      }
    }
    return scored;
  }

  static final double[] WIDE = {-2, -1, 0, 1, 2};

  /**
   * Scores out of 125: {-2,-1,0,1,2} x tolerance on aim, hood and rps (a wider ring for ranking).
   */
  static int widePerturbedScore(FuelLaunchCalculator calc, Translation2d launch, Setpoint s) {
    int scored = 0;
    for (double a : WIDE) {
      for (double h : WIDE) {
        for (double r : WIDE) {
          if (SIM.get()
              .shoot(
                  calc,
                  launch,
                  s.yawDeg() + a * AIM_TOL_DEG,
                  s.hoodDeg() + h * HOOD_TOL_DEG,
                  s.rps() * (1 + r * RPS_TOL_FRACTION))
              .scored()) {
            scored++;
          }
        }
      }
    }
    return scored;
  }

  /** Contiguous scoring window around nominal along one axis; returns {low, high} offsets. */
  static double[] window(java.util.function.DoublePredicate scoresAt, double step, double limit) {
    if (!scoresAt.test(0)) {
      return new double[] {Double.NaN, Double.NaN};
    }
    double lo = 0;
    while (lo - step >= -limit - 1e-9 && scoresAt.test(lo - step)) {
      lo -= step;
    }
    double hi = 0;
    while (hi + step <= limit + 1e-9 && scoresAt.test(hi + step)) {
      hi += step;
    }
    return new double[] {lo, hi};
  }

  // ---------------------------------------------------------------- region / grids

  static boolean inRegion(Translation2d p) {
    return p.getX() >= REGION_X_MIN
        && p.getX() <= REGION_X_MAX
        && p.getY() >= REGION_MARGIN_Y
        && p.getY() <= FIELD_WIDTH - REGION_MARGIN_Y;
  }

  /**
   * Launch point at distance d from the robot's aim point (so d is the shot-table key). Bearing 0 =
   * straight back from the hub toward the blue wall; + toward +y.
   */
  static Translation2d polar(double d, double bearingDeg) {
    double b = Math.toRadians(bearingDeg);
    return AIM_POINT.plus(new Translation2d(-d * Math.cos(b), d * Math.sin(b)));
  }

  static List<double[]> sweepGrid() {
    List<double[]> pts = new ArrayList<>();
    for (int i = 0; i <= 94; i++) {
      double d = Math.round((1.0 + 0.05 * i) * 1000) / 1000.0;
      for (int b = -90; b <= 90; b += 5) {
        if (inRegion(polar(d, b))) {
          pts.add(new double[] {d, b});
        }
      }
    }
    return pts;
  }

  static List<Integer> bearingsAt(double d) {
    List<Integer> out = new ArrayList<>();
    for (int b = -90; b <= 90; b += 5) {
      if (inRegion(polar(d, b))) {
        out.add(b);
      }
    }
    return out;
  }

  private static String env(String name, String fallback) {
    String v = System.getenv(name);
    return v == null || v.isBlank() ? fallback : v;
  }

  private static String f(double v) {
    return String.format(Locale.ROOT, "%.4f", v);
  }

  // ---------------------------------------------------------------- modes

  /** Exports the mapping the sim derives from the current (unedited) table and self-checks. */
  @Test
  void export() throws Exception {
    assumeTrue(MODE.equals("export"));
    Files.createDirectories(DIR);
    Object derived = derivedCalibration();
    List<double[]> hood = calibrationPoints(derived, "hoodAnglePoints");
    List<double[]> rps = calibrationPoints(derived, "flywheelSpeedPoints");

    ObjectNode root = JSON.createObjectNode();
    root.put(
        "description",
        "Frozen copy of FuelLaunchCalculator.LaunchCalibration.fromShotTable() derived from the"
            + " unedited src/main/deploy/shot_table.json. Values are unscaled; the sim multiplies"
            + " exit speed by EXIT_VELOCITY_SCALE at lookup.");
    root.put("sourceTableSha256", sha256(DEPLOY_TABLE));
    root.put("exitVelocityScaleAtExport", Constants.SimConstants.Fuel.EXIT_VELOCITY_SCALE);
    ArrayNode h = root.putArray("hoodDegToLaunchAngleDeg");
    hood.forEach(p -> h.addArray().add(p[0]).add(p[1]));
    ArrayNode r = root.putArray("rpsToExitSpeedMetersPerSecond");
    rps.forEach(p -> r.addArray().add(p[0]).add(p[1]));
    JSON.writerWithDefaultPrettyPrinter().writeValue(FROZEN.toFile(), root);

    // The rebuilt calibration must reproduce the derived one exactly.
    FuelLaunchCalculator live = calculatorFor(derived);
    FuelLaunchCalculator frozen = calculatorFor(frozenCalibration());
    Pose2d pose = new Pose2d(2, 3, Rotation2d.kZero);
    for (double hd = 8; hd <= 32; hd += 0.05) {
      for (double rv = 30; rv <= 80; rv += 0.5) {
        var a =
            live.calculateTurretLaunch(pose, new ChassisSpeeds(), Translation2d.kZero, 10, hd, rv);
        var b =
            frozen.calculateTurretLaunch(
                pose, new ChassisSpeeds(), Translation2d.kZero, 10, hd, rv);
        assertEquals(a.velocity(), b.velocity());
      }
    }

    // The table lookup mirror must match GetAdjustedShot exactly for the deploy table.
    Table table = new Table(DEPLOY_TABLE);
    for (double d = 0.5; d <= 7.0; d += 0.01) {
      Distancer mine = table.get(d);
      Distancer theirs = GetAdjustedShot.getTableShot(d);
      assertEquals(theirs.hoodAngleDeg(), mine.hoodAngleDeg(), 1e-12);
      assertEquals(theirs.flywheelRps(), mine.flywheelRps(), 1e-12);
      assertEquals(theirs.tofSec(), mine.tofSec(), 1e-12);
    }

    // Determinism: the same shot twice gives identical results.
    Translation2d launch = polar(3.0, 20);
    Setpoint s = robotSetpoint(table, launch);
    ShotResult first = SIM.get().shoot(frozen, launch, s.yawDeg(), s.hoodDeg(), s.rps());
    ShotResult second = SIM.get().shoot(frozen, launch, s.yawDeg(), s.hoodDeg(), s.rps());
    assertEquals(first, second);

    // Teleport-based score detection agrees with the hub's own counter.
    FuelSim.Hub.BLUE_HUB.resetScore();
    int expected = 0;
    for (double d = 1.0; d <= 5.7; d += 0.1) {
      Translation2d p = polar(d, 0);
      Setpoint sp = robotSetpoint(table, p);
      if (SIM.get().shoot(frozen, p, sp.yawDeg(), sp.hoodDeg(), sp.rps()).scored()) {
        expected++;
      }
    }
    assertEquals(expected, FuelSim.Hub.BLUE_HUB.getScore());
    System.out.println("export ok; scored-check " + expected);
  }

  /** Sweeps launch positions with a table (STA_TABLE) against a calibration (STA_CAL). */
  @Test
  void sweep() throws Exception {
    assumeTrue(MODE.equals("sweep"));
    Table table = new Table(Path.of(env("STA_TABLE", DEPLOY_TABLE.toString())));
    FuelLaunchCalculator calc = selectedCalculator();
    List<double[]> grid = sweepGrid();
    String[] lines = new String[grid.size()];
    IntStream.range(0, grid.size())
        .parallel()
        .forEach(
            i -> {
              double d = grid.get(i)[0];
              double b = grid.get(i)[1];
              Translation2d p = polar(d, b);
              Setpoint s = robotSetpoint(table, p);
              ShotResult r = SIM.get().shoot(calc, p, s.yawDeg(), s.hoodDeg(), s.rps());
              int pert = perturbedScore(calc, p, s);
              lines[i] =
                  String.join(
                      ",",
                      f(d),
                      f(b),
                      f(p.getX()),
                      f(p.getY()),
                      f(s.robotDistance()),
                      f(s.hoodDeg()),
                      f(s.rps()),
                      r.scored() ? "1" : "0",
                      f(r.tof()),
                      f(r.crossAlong()),
                      f(r.crossLateral()),
                      r.bounced() ? "1" : "0",
                      f(r.apex()),
                      Integer.toString(pert));
            });
    Path out = DIR.resolve(env("STA_OUT", "sweep.csv"));
    try (PrintWriter w = new PrintWriter(Files.newBufferedWriter(out))) {
      w.println(
          "d,bearing,x,y,robotDist,hood,rps,scored,tof,crossAlong,crossLateral,bounced,apex,pert27");
      for (String l : lines) {
        w.println(l);
      }
    }
    System.out.println("wrote " + out + " (" + lines.length + " positions)");
  }

  /**
   * Candidate search: for each distance, every (hood, rps) on a grid, shot from the bearing closest
   * to straight-on. Scoring candidates also get the 27-shot perturbation count and a nominal check
   * at every in-region bearing for that distance.
   */
  @Test
  void search() throws Exception {
    assumeTrue(MODE.equals("search"));
    FuelLaunchCalculator calc = selectedCalculator();
    double hoodMin = Double.parseDouble(env("STA_HOOD_MIN", "13"));
    double hoodMax = Double.parseDouble(env("STA_HOOD_MAX", "25.5"));
    double rpsMin = Double.parseDouble(env("STA_RPS_MIN", "36"));
    double rpsMax = Double.parseDouble(env("STA_RPS_MAX", "70"));
    double dMin = Double.parseDouble(env("STA_D_MIN", "1.0"));
    double dMax = Double.parseDouble(env("STA_D_MAX", "5.7"));
    double dStep = Double.parseDouble(env("STA_D_STEP", "0.05"));
    List<Double> hoods = new ArrayList<>();
    for (double h = hoodMin; h <= hoodMax + 1e-9; h += 0.25) {
      hoods.add(h);
    }
    List<Double> rpss = new ArrayList<>();
    for (double r = rpsMin; r <= rpsMax + 1e-9; r += 0.25) {
      rpss.add(r);
    }

    Path out = DIR.resolve(env("STA_OUT", "search.csv"));
    try (PrintWriter w = new PrintWriter(Files.newBufferedWriter(out))) {
      w.println(
          "d,hood,rps,bearing,tof,crossAlong,crossLateral,bounced,pert27,bearingsScored,bearingsTotal");
      for (double d = dMin; d <= dMax + 1e-9; d += dStep) {
        double dd = Math.round(d * 1000) / 1000.0;
        List<Integer> bearings = bearingsAt(dd);
        if (bearings.isEmpty() && env("STA_IGNORE_REGION", "").equals("1")) {
          bearings =
              List.of(0); // hypothetical straight-on shot, used only for rows past the region
        }
        if (bearings.isEmpty()) {
          continue;
        }
        List<Integer> bearingList = bearings;
        int rep = bearingList.stream().min(Comparator.comparingInt(Math::abs)).get();
        Translation2d launch = polar(dd, rep);
        String[] rows = new String[hoods.size() * rpss.size()];
        IntStream.range(0, rows.length)
            .parallel()
            .forEach(
                i -> {
                  double h = hoods.get(i / rpss.size());
                  double r = rpss.get(i % rpss.size());
                  double yaw = AIM_POINT.minus(launch).getAngle().getDegrees();
                  ShotResult res = SIM.get().shoot(calc, launch, yaw, h, r);
                  if (!res.scored()) {
                    return;
                  }
                  Setpoint s = new Setpoint(yaw, h, r, dd);
                  int pert = perturbedScore(calc, launch, s);
                  int ok = 0;
                  for (int b : bearingList) {
                    Translation2d p = polar(dd, b);
                    double y = AIM_POINT.minus(p).getAngle().getDegrees();
                    if (SIM.get().shoot(calc, p, y, h, r).scored()) {
                      ok++;
                    }
                  }
                  rows[i] =
                      String.join(
                          ",",
                          f(dd),
                          f(h),
                          f(r),
                          Integer.toString(rep),
                          f(res.tof()),
                          f(res.crossAlong()),
                          f(res.crossLateral()),
                          res.bounced() ? "1" : "0",
                          Integer.toString(pert),
                          Integer.toString(ok),
                          Integer.toString(bearingList.size()));
                });
        for (String row : rows) {
          if (row != null) {
            w.println(row);
          }
        }
        w.flush();
        System.out.println("search d=" + dd);
      }
    }
  }

  /**
   * Per-distance detail for a table: nominal result, 27-shot perturbation count and one-axis
   * scoring windows, at every in-region bearing. Distances come from STA_DISTANCES (comma list) or
   * default to every table row.
   */
  @Test
  void rows() throws Exception {
    assumeTrue(MODE.equals("rows"));
    Table table = new Table(Path.of(env("STA_TABLE", DEPLOY_TABLE.toString())));
    FuelLaunchCalculator calc = selectedCalculator();
    List<Double> distances = new ArrayList<>();
    String list = System.getenv("STA_DISTANCES");
    if (list != null && !list.isBlank()) {
      for (String s : list.split(",")) {
        distances.add(Double.parseDouble(s.trim()));
      }
    } else {
      table.rows.forEach(r -> distances.add(r.d));
    }
    List<String> lines = new ArrayList<>();
    for (double d : distances) {
      for (int b : bearingsAt(d)) {
        lines.add(d + "," + b);
      }
    }
    String[] out = new String[lines.size()];
    IntStream.range(0, lines.size())
        .parallel()
        .forEach(
            i -> {
              String[] parts = lines.get(i).split(",");
              double d = Double.parseDouble(parts[0]);
              int b = Integer.parseInt(parts[1]);
              Translation2d p = polar(d, b);
              Setpoint s = robotSetpoint(table, p);
              Sim sim = SIM.get();
              ShotResult r = sim.shoot(calc, p, s.yawDeg(), s.hoodDeg(), s.rps());
              int pert = perturbedScore(calc, p, s);
              int ext = widePerturbedScore(calc, p, s);
              double[] rpsWin =
                  window(
                      x -> sim.shoot(calc, p, s.yawDeg(), s.hoodDeg(), s.rps() * (1 + x)).scored(),
                      0.0025,
                      0.15);
              double[] hoodWin =
                  window(
                      x -> sim.shoot(calc, p, s.yawDeg(), s.hoodDeg() + x, s.rps()).scored(),
                      0.1,
                      6);
              double[] aimWin =
                  window(
                      x -> sim.shoot(calc, p, s.yawDeg() + x, s.hoodDeg(), s.rps()).scored(),
                      0.1,
                      15);
              out[i] =
                  String.join(
                      ",",
                      f(d),
                      Integer.toString(b),
                      f(s.robotDistance()),
                      f(s.hoodDeg()),
                      f(s.rps()),
                      r.scored() ? "1" : "0",
                      f(r.tof()),
                      f(r.crossAlong()),
                      f(r.crossLateral()),
                      r.bounced() ? "1" : "0",
                      Integer.toString(pert),
                      Integer.toString(ext),
                      f(rpsWin[0] * 100),
                      f(rpsWin[1] * 100),
                      f(hoodWin[0]),
                      f(hoodWin[1]),
                      f(aimWin[0]),
                      f(aimWin[1]));
            });
    Path path = DIR.resolve(env("STA_OUT", "rows.csv"));
    try (PrintWriter w = new PrintWriter(Files.newBufferedWriter(path))) {
      w.println(
          "d,bearing,robotDist,hood,rps,scored,tof,crossAlong,crossLateral,bounced,pert27,ext125,"
              + "rpsLoPct,rpsHiPct,hoodLo,hoodHi,aimLo,aimHi");
      for (String l : out) {
        w.println(l);
      }
    }
    System.out.println("wrote " + path);
  }

  private static String sha256(Path p) throws Exception {
    var md = java.security.MessageDigest.getInstance("SHA-256");
    byte[] digest = md.digest(Files.readAllBytes(p));
    StringBuilder sb = new StringBuilder();
    for (byte b : digest) {
      sb.append(String.format("%02x", b));
    }
    return sb.toString();
  }
}
