package frc.robot.util;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public record Distancer(double hoodAngleDeg, double flywheelRps, double tofSec) {

  public Distancer interpolate(Distancer endValue, double t) {
    t = clamp01(t);

    double hood = lerp(this.hoodAngleDeg, endValue.hoodAngleDeg, t);
    double rps = lerp(this.flywheelRps, endValue.flywheelRps, t);
    double tof = lerp(this.tofSec, endValue.tofSec, t);

    return new Distancer(hood, rps, tof);
  }

  public static class DistancerFile {
    public Units units;
    public List<Row> points;
  }

  public static class Units {
    public String distance; // "m"
    public String hood; // "deg"
    public String flywheel; // "rps"
    public String tof; // "s"
  }

  public static class Row {
    public double d; // distance meters
    public double hoodDeg; // hood degrees
    public double rps; // flywheel RPS
    public double tof; // seconds
  }

  public static List<Row> loadRowsFromDeploy(String filename) {
    try {
      File file = new File(Filesystem.getDeployDirectory(), filename);
      ObjectMapper mapper = new ObjectMapper();
      DistancerFile data = mapper.readValue(file, DistancerFile.class);

      if (data == null || data.points == null) return List.of();

      ArrayList<Row> rows = new ArrayList<>(data.points);
      rows.sort(Comparator.comparingDouble(r -> r.d));
      return rows;

    } catch (Exception e) {
      System.err.println("[Distancer] Failed to load " + filename + ": " + e.getMessage());
      return List.of();
    }
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }

  private static double clamp01(double x) {
    return Math.max(0.0, Math.min(1.0, x));
  }
}
