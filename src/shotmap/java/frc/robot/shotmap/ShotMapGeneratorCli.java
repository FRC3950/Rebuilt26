package frc.robot.shotmap;

import static frc.robot.Constants.SubsystemConstants.Turret.maxFlywheelRps;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import frc.robot.util.ShotTable2d;
import frc.robot.util.ShotTableTarget;
import java.io.ByteArrayOutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.Locale;

public final class ShotMapGeneratorCli {
  private static final Path DEFAULT_GYMSIM_SCENARIO =
      Path.of(".gymsim/tests/shotmap-generated-sweep.yml");
  private static final String GYMSIM_BIN_DIR = "/Users/cjbrandi/GymSim/bin";

  private ShotMapGeneratorCli() {}

  public static void main(String[] args) throws Exception {
    CliOptions options = CliOptions.parse(args);
    ShotMapConfig config = createConfig(options.mode(), options.target(), options.flywheelScale());
    ShotMapGenerator.Output output = new ShotMapGenerator(config).generate();

    Path deployShotTablePath = deployPathFor(options.target());
    Path stagedTable =
        stageTable(output.table(), config.mode(), config.target(), options.stagingPath());
    validateWithGymSim(
        stagedTable, options.output(), deployShotTablePath, options.gymsimScenario());
    publishTable(stagedTable, options.output());
    writeDiagnostics(output, options.diagnostics());
    System.out.println(
        "Generated "
            + config.mode()
            + " "
            + config.target().fileTargetName()
            + " shot map: "
            + output.table().distanceMetersList().size()
            + " distances x "
            + output.table().radialVelocityMetersPerSecondList().size()
            + " radial velocities -> "
            + options.output());
  }

  private static Path stageTable(
      ShotTable2d table, String mode, ShotTableTarget target, Path stagingPath) throws Exception {
    writeTable(table, mode, target, stagingPath);
    return stagingPath;
  }

  private static void writeTable(
      ShotTable2d table, String mode, ShotTableTarget target, Path outputPath) throws Exception {
    Files.createDirectories(outputPath.toAbsolutePath().getParent());
    ObjectMapper mapper = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);
    mapper.writeValue(outputPath.toFile(), table.toFileFormat(mode, target));
  }

  private static void validateWithGymSim(
      Path stagedTable, Path outputPath, Path deployShotTablePath, Path scenarioPath)
      throws Exception {
    Path backupPath = null;
    boolean deployHadTable = Files.isRegularFile(deployShotTablePath);

    if (deployHadTable) {
      backupPath = Files.createTempFile("shot_table_2d_backup", ".json");
      Files.copy(deployShotTablePath, backupPath, StandardCopyOption.REPLACE_EXISTING);
    }

    try {
      Files.createDirectories(deployShotTablePath.toAbsolutePath().getParent());
      Files.copy(stagedTable, deployShotTablePath, StandardCopyOption.REPLACE_EXISTING);
      GymSimResult result = runGymSimScenario(scenarioPath);
      System.out.print(result.output());
      if (result.exitCode() != 0) {
        throw new IllegalStateException(
            "GymSim validation failed for generated shot map with exit code " + result.exitCode());
      }
    } catch (Exception ex) {
      restoreDeployTable(deployShotTablePath, backupPath, deployHadTable);
      throw ex;
    }

    if (!deployShotTablePath
        .toAbsolutePath()
        .normalize()
        .equals(outputPath.toAbsolutePath().normalize())) {
      restoreDeployTable(deployShotTablePath, backupPath, deployHadTable);
    } else if (backupPath != null) {
      Files.deleteIfExists(backupPath);
    }
  }

  private static GymSimResult runGymSimScenario(Path scenarioPath) throws Exception {
    if (!Files.isRegularFile(scenarioPath)) {
      throw new IllegalArgumentException("GymSim scenario does not exist: " + scenarioPath);
    }

    ProcessBuilder builder =
        new ProcessBuilder(
            "/bin/bash",
            "-lc",
            "export PATH=\""
                + GYMSIM_BIN_DIR
                + ":$PATH\"; gymsim run "
                + shellQuote(scenarioPath.toString()));
    builder.redirectErrorStream(true);
    Process process = builder.start();
    ByteArrayOutputStream output = new ByteArrayOutputStream();
    process.getInputStream().transferTo(output);
    int exitCode = process.waitFor();
    return new GymSimResult(exitCode, output.toString());
  }

  private static void restoreDeployTable(
      Path deployShotTablePath, Path backupPath, boolean deployHadTable) throws Exception {
    if (deployHadTable) {
      Files.copy(backupPath, deployShotTablePath, StandardCopyOption.REPLACE_EXISTING);
      Files.deleteIfExists(backupPath);
    } else {
      Files.deleteIfExists(deployShotTablePath);
    }
  }

  private static void publishTable(Path stagedTable, Path outputPath) throws Exception {
    Files.createDirectories(outputPath.toAbsolutePath().getParent());
    Files.copy(stagedTable, outputPath, StandardCopyOption.REPLACE_EXISTING);
  }

  private static String shellQuote(String value) {
    return "'" + value.replace("'", "'\"'\"'") + "'";
  }

  private static void writeDiagnostics(ShotMapGenerator.Output output, Path diagnosticsDir)
      throws Exception {
    Files.createDirectories(diagnosticsDir);
    ObjectMapper mapper = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);
    mapper.writeValue(diagnosticsDir.resolve("selected_shots.json").toFile(), output.diagnostics());

    StringBuilder csv = new StringBuilder();
    csv.append(
        "distanceMeters,radialVelocityMetersPerSecond,hoodDeg,flywheelRps,tofSec,robustnessMarginMeters,postBounceHorizontalVelocityMetersPerSecond,maxHeightMeters,firstContactXFieldMeters,firstContactYFieldMeters,firstContactSurfaceHeightMeters,firstContactOnBump,baseScored\n");
    for (ShotMapGenerator.DiagnosticRow row : output.diagnostics()) {
      csv.append(
          String.format(
              Locale.US,
              "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%s%n",
              row.distanceMeters(),
              row.radialVelocityMetersPerSecond(),
              row.hoodDeg(),
              row.flywheelRps(),
              row.tofSec(),
              row.robustnessMarginMeters(),
              row.postBounceHorizontalVelocityMetersPerSecond(),
              row.maxHeightMeters(),
              row.firstContactXFieldMeters(),
              row.firstContactYFieldMeters(),
              row.firstContactSurfaceHeightMeters(),
              row.firstContactOnBump(),
              row.baseScored()));
    }
    Files.writeString(diagnosticsDir.resolve("selected_shots.csv"), csv.toString());
    Files.writeString(diagnosticsDir.resolve("surface_plot_data.csv"), csv.toString());
  }

  private record GymSimResult(int exitCode, String output) {}

  static ShotMapConfig createConfigForTest(
      String mode, ShotTableTarget target, double flywheelScale) {
    return createConfig(mode, target, flywheelScale);
  }

  private static ShotMapConfig createConfig(
      String mode, ShotTableTarget target, double flywheelScale) {
    ShotMapConfig config =
        ("final".equals(mode) ? ShotMapConfig.finall() : ShotMapConfig.preview())
            .withTarget(target)
            .withFlywheelScale(flywheelScale);
    if (target == ShotTableTarget.FERRY) {
      config =
          config
              .withDistanceRange(3.0, 9.5, "final".equals(mode) ? 0.25 : 0.5)
              .withRadialVelocityRange(-4.5, 4.5, "final".equals(mode) ? 0.75 : 1.5)
              .withCoarseSearch(
                  config.coarseSearch().minHoodDeg(),
                  config.coarseSearch().maxHoodDeg(),
                  config.coarseSearch().hoodStepDeg(),
                  config.coarseSearch().minFlywheelRps(),
                  maxFlywheelRps,
                  config.coarseSearch().flywheelStepRps());
    }
    return config;
  }

  private record CliOptions(
      ShotTableTarget target,
      String mode,
      Path output,
      Path diagnostics,
      double flywheelScale,
      Path stagingPath,
      Path gymsimScenario) {
    static CliOptions parse(String[] args) {
      ShotTableTarget target = ShotTableTarget.HUB;
      String mode = "preview";
      Path output = null;
      Path diagnostics = null;
      double flywheelScale = 1.0;
      Path stagingPath = null;
      Path gymsimScenario = DEFAULT_GYMSIM_SCENARIO;

      for (int i = 0; i < args.length; i++) {
        switch (args[i]) {
          case "--target" -> target =
              ShotTableTarget.fromCliName(requireValue(args, ++i, "--target"));
          case "--mode" -> mode = requireValue(args, ++i, "--mode");
          case "--output" -> output = Path.of(requireValue(args, ++i, "--output"));
          case "--diagnostics" -> diagnostics = Path.of(requireValue(args, ++i, "--diagnostics"));
          case "--flywheel-scale" -> flywheelScale =
              Double.parseDouble(requireValue(args, ++i, "--flywheel-scale"));
          case "--staging-output" -> stagingPath =
              Path.of(requireValue(args, ++i, "--staging-output"));
          case "--gymsim-scenario" -> gymsimScenario =
              Path.of(requireValue(args, ++i, "--gymsim-scenario"));
          default -> throw new IllegalArgumentException("Unknown argument: " + args[i]);
        }
      }

      if (!mode.equals("preview") && !mode.equals("final")) {
        throw new IllegalArgumentException("--mode must be preview or final");
      }
      if (!Double.isFinite(flywheelScale) || flywheelScale <= 0.0) {
        throw new IllegalArgumentException("--flywheel-scale must be positive and finite");
      }
      if (stagingPath == null) {
        stagingPath =
            Path.of(
                "build/shotmap/"
                    + mode
                    + "/"
                    + target.name().toLowerCase(Locale.US)
                    + "/staged_"
                    + target.deployFilename());
      }
      if (output == null) {
        output =
            Path.of(
                "build/shotmap/"
                    + mode
                    + "/"
                    + target.name().toLowerCase(Locale.US)
                    + "/"
                    + target.deployFilename());
      }
      if (diagnostics == null) {
        diagnostics =
            Path.of(
                "build/shotmap/"
                    + mode
                    + "/"
                    + target.name().toLowerCase(Locale.US)
                    + "/diagnostics");
      }
      return new CliOptions(
          target, mode, output, diagnostics, flywheelScale, stagingPath, gymsimScenario);
    }

    private static String requireValue(String[] args, int index, String flag) {
      if (index >= args.length) {
        throw new IllegalArgumentException(flag + " requires a value");
      }
      return args[index];
    }
  }

  private static Path deployPathFor(ShotTableTarget target) {
    return Path.of("src/main/deploy", target.deployFilename());
  }
}
