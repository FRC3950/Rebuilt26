package frc.robot.util;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

public final class ShotTable2d {
  public static final String DEFAULT_DEPLOY_FILENAME = "shot_table_2d.json";

  private final double[] distanceMeters;
  private final double[] radialVelocityMetersPerSecond;
  private final Sample[][] samples;

  public ShotTable2d(
      double[] distanceMeters, double[] radialVelocityMetersPerSecond, Sample[][] samples) {
    validate(distanceMeters, radialVelocityMetersPerSecond, samples);
    this.distanceMeters = distanceMeters.clone();
    this.radialVelocityMetersPerSecond = radialVelocityMetersPerSecond.clone();
    this.samples = copySamples(samples);
  }

  public double[] distanceMeters() {
    return distanceMeters.clone();
  }

  public double[] radialVelocityMetersPerSecond() {
    return radialVelocityMetersPerSecond.clone();
  }

  public List<Double> distanceMetersList() {
    return toList(distanceMeters);
  }

  public List<Double> radialVelocityMetersPerSecondList() {
    return toList(radialVelocityMetersPerSecond);
  }

  public Sample[][] samples() {
    return copySamples(samples);
  }

  public Distancer interpolate(double distanceMeters, double radialVelocityMetersPerSecond) {
    Sample sample = interpolateSample(distanceMeters, radialVelocityMetersPerSecond);
    return new Distancer(sample.hoodDeg(), sample.flywheelRps(), sample.tofSec());
  }

  public Distancer interpolateClamped(double distanceMeters, double radialVelocityMetersPerSecond) {
    Sample sample = interpolateSampleClamped(distanceMeters, radialVelocityMetersPerSecond);
    return new Distancer(sample.hoodDeg(), sample.flywheelRps(), sample.tofSec());
  }

  public Sample interpolateSample(double distanceMeters, double radialVelocityMetersPerSecond) {
    Bracket distance = bracket(this.distanceMeters, distanceMeters);
    Bracket radialVelocity =
        bracket(this.radialVelocityMetersPerSecond, radialVelocityMetersPerSecond);

    Sample nearSlow = samples[distance.lowerIndex()][radialVelocity.lowerIndex()];
    Sample nearFast = samples[distance.lowerIndex()][radialVelocity.upperIndex()];
    Sample farSlow = samples[distance.upperIndex()][radialVelocity.lowerIndex()];
    Sample farFast = samples[distance.upperIndex()][radialVelocity.upperIndex()];

    Sample near = nearSlow.interpolate(nearFast, radialVelocity.t());
    Sample far = farSlow.interpolate(farFast, radialVelocity.t());
    return near.interpolate(far, distance.t());
  }

  public Sample interpolateSampleClamped(
      double distanceMeters, double radialVelocityMetersPerSecond) {
    return interpolateSample(
        clamp(
            distanceMeters,
            this.distanceMeters[0],
            this.distanceMeters[this.distanceMeters.length - 1]),
        clamp(
            radialVelocityMetersPerSecond,
            this.radialVelocityMetersPerSecond[0],
            this.radialVelocityMetersPerSecond[this.radialVelocityMetersPerSecond.length - 1]));
  }

  public static Optional<ShotTable2d> loadFromDeploy() {
    return loadFromDeploy(DEFAULT_DEPLOY_FILENAME);
  }

  public static Optional<ShotTable2d> loadFromDeploy(String filename) {
    try {
      File file = new File(Filesystem.getDeployDirectory(), filename);
      if (!file.isFile()) {
        return Optional.empty();
      }
      ObjectMapper mapper = new ObjectMapper();
      FileFormat data = mapper.readValue(file, FileFormat.class);
      return Optional.of(data.toTable());
    } catch (Exception e) {
      System.err.println("[ShotTable2d] Failed to load " + filename + ": " + e.getMessage());
      return Optional.empty();
    }
  }

  public FileFormat toFileFormat(String mode) {
    return toFileFormat(mode, ShotTableTarget.HUB);
  }

  public FileFormat toFileFormat(String mode, ShotTableTarget target) {
    FileFormat file = new FileFormat();
    file.version = 1;
    file.target = target.fileTargetName();
    file.mode = mode;
    file.units = new Units();
    file.units.distance = "m";
    file.units.radialVelocity = "m/s";
    file.units.hood = "deg";
    file.units.flywheel = "rps";
    file.units.tof = "s";
    file.units.robustnessMargin = "m";
    file.distanceMeters = distanceMetersList();
    file.radialVelocityMetersPerSecond = radialVelocityMetersPerSecondList();
    file.points = new ArrayList<>();
    for (Sample[] sampleRow : samples) {
      file.points.add(List.of(sampleRow.clone()));
    }
    return file;
  }

  @Override
  public boolean equals(Object other) {
    if (!(other instanceof ShotTable2d table)) {
      return false;
    }
    return Arrays.equals(distanceMeters, table.distanceMeters)
        && Arrays.equals(radialVelocityMetersPerSecond, table.radialVelocityMetersPerSecond)
        && Arrays.deepEquals(samples, table.samples);
  }

  @Override
  public int hashCode() {
    int result = Arrays.hashCode(distanceMeters);
    result = 31 * result + Arrays.hashCode(radialVelocityMetersPerSecond);
    result = 31 * result + Arrays.deepHashCode(samples);
    return result;
  }

  private static void validate(
      double[] distanceMeters, double[] radialVelocityMetersPerSecond, Sample[][] samples) {
    Objects.requireNonNull(distanceMeters);
    Objects.requireNonNull(radialVelocityMetersPerSecond);
    Objects.requireNonNull(samples);
    if (distanceMeters.length == 0 || radialVelocityMetersPerSecond.length == 0) {
      throw new IllegalArgumentException("shot table axes must not be empty");
    }
    requireStrictlyIncreasing(distanceMeters, "distanceMeters");
    requireStrictlyIncreasing(radialVelocityMetersPerSecond, "radialVelocityMetersPerSecond");
    if (samples.length != distanceMeters.length) {
      throw new IllegalArgumentException("sample row count must match distance axis");
    }
    for (Sample[] row : samples) {
      if (row == null || row.length != radialVelocityMetersPerSecond.length) {
        throw new IllegalArgumentException("sample column count must match radial velocity axis");
      }
      for (Sample sample : row) {
        if (sample == null) {
          throw new IllegalArgumentException("shot table samples must not be null");
        }
      }
    }
  }

  private static void requireStrictlyIncreasing(double[] values, String name) {
    for (int i = 0; i < values.length; i++) {
      if (!Double.isFinite(values[i])) {
        throw new IllegalArgumentException(name + " contains a non-finite value");
      }
      if (i > 0 && values[i] <= values[i - 1]) {
        throw new IllegalArgumentException(name + " must be strictly increasing");
      }
    }
  }

  private static Sample[][] copySamples(Sample[][] source) {
    Sample[][] copy = new Sample[source.length][];
    for (int i = 0; i < source.length; i++) {
      copy[i] = source[i].clone();
    }
    return copy;
  }

  private static List<Double> toList(double[] values) {
    ArrayList<Double> list = new ArrayList<>(values.length);
    for (double value : values) {
      list.add(value);
    }
    return List.copyOf(list);
  }

  private static Bracket bracket(double[] axis, double value) {
    if (axis.length == 1) {
      return new Bracket(0, 0, 0.0);
    }
    int lowerIndex = 0;
    int upperIndex = axis.length - 1;

    if (value <= axis[0]) {
      lowerIndex = 0;
      upperIndex = 1;
    } else if (value >= axis[axis.length - 1]) {
      lowerIndex = axis.length - 2;
      upperIndex = axis.length - 1;
    } else {
      for (int i = 0; i < axis.length - 1; i++) {
        if (value >= axis[i] && value <= axis[i + 1]) {
          lowerIndex = i;
          upperIndex = i + 1;
          break;
        }
      }
    }

    double span = axis[upperIndex] - axis[lowerIndex];
    double t = span == 0.0 ? 0.0 : (value - axis[lowerIndex]) / span;
    return new Bracket(lowerIndex, upperIndex, t);
  }

  private record Bracket(int lowerIndex, int upperIndex, double t) {}

  public record Sample(double hoodDeg, double flywheelRps, double tofSec, double robustnessMargin) {
    public Sample interpolate(Sample endValue, double t) {
      return new Sample(
          lerp(hoodDeg, endValue.hoodDeg, t),
          lerp(flywheelRps, endValue.flywheelRps, t),
          lerp(tofSec, endValue.tofSec, t),
          lerp(robustnessMargin, endValue.robustnessMargin, t));
    }
  }

  public static class FileFormat {
    public int version;
    public String target;
    public String mode;
    public Units units;
    public List<Double> distanceMeters;
    public List<Double> radialVelocityMetersPerSecond;
    public List<List<Sample>> points;

    ShotTable2d toTable() {
      if (distanceMeters == null || radialVelocityMetersPerSecond == null || points == null) {
        throw new IllegalArgumentException("shot table file is missing axes or points");
      }
      Sample[][] tableSamples = new Sample[points.size()][];
      for (int i = 0; i < points.size(); i++) {
        tableSamples[i] = points.get(i).toArray(Sample[]::new);
      }
      return new ShotTable2d(
          distanceMeters.stream().mapToDouble(Double::doubleValue).toArray(),
          radialVelocityMetersPerSecond.stream().mapToDouble(Double::doubleValue).toArray(),
          tableSamples);
    }
  }

  public static class Units {
    public String distance;
    public String radialVelocity;
    public String hood;
    public String flywheel;
    public String tof;
    public String robustnessMargin;
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
