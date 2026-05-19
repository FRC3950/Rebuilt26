package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public final class BatteryLogger {
  private static final BatteryLogger instance = new BatteryLogger();
  private static final int DETAIL_LOG_PERIOD_LOOPS = 5;
  private static final int PDH_LOG_PERIOD_LOOPS = 5;

  private final Map<String, Double> currentReports = new HashMap<>();
  private final Map<String, Double> currentRollups = new HashMap<>();
  private final Map<String, String[]> rollupKeysByReportKey = new HashMap<>();
  private final Map<String, Double> energyJoules = new HashMap<>();
  private final Set<String> previouslyLoggedRollups = new HashSet<>();
  private final PowerDistribution powerDistribution;
  private double totalReportedEnergyJoules = 0.0;
  private int loopCounter = 0;

  private BatteryLogger() {
    powerDistribution =
        Constants.currentMode == Constants.Mode.REAL ? new PowerDistribution() : null;
  }

  public static BatteryLogger getInstance() {
    return instance;
  }

  public static void reportCurrentUsage(String key, double... currentAmps) {
    instance.report(key, currentAmps);
  }

  public void periodicAfterScheduler() {
    double voltageVolts = getBatteryVoltage();
    currentRollups.clear();
    double totalReportedCurrentAmps = 0.0;

    for (var entry : currentReports.entrySet()) {
      double currentAmps = sanitize(entry.getValue());
      totalReportedCurrentAmps += currentAmps;
      addRollups(currentRollups, entry.getKey(), currentAmps);
    }

    double totalReportedPowerWatts = totalReportedCurrentAmps * voltageVolts;
    totalReportedEnergyJoules += totalReportedPowerWatts * Constants.loopPeriodSecs;
    boolean logDetails = loopCounter % DETAIL_LOG_PERIOD_LOOPS == 0;

    for (var rollup : currentRollups.entrySet()) {
      String key = rollup.getKey();
      double currentAmps = sanitize(rollup.getValue());
      double powerWatts = currentAmps * voltageVolts;
      double totalEnergyJoules =
          energyJoules.getOrDefault(key, 0.0) + powerWatts * Constants.loopPeriodSecs;
      energyJoules.put(key, totalEnergyJoules);

      if (logDetails) {
        Logger.recordOutput("EnergyLogger/Current/" + key, currentAmps);
        Logger.recordOutput("EnergyLogger/Power/" + key, powerWatts);
        Logger.recordOutput("EnergyLogger/Energy/" + key, totalEnergyJoules);
      }
    }

    if (logDetails) {
      for (String staleKey : previouslyLoggedRollups) {
        if (!currentRollups.containsKey(staleKey)) {
          Logger.recordOutput("EnergyLogger/Current/" + staleKey, 0.0);
          Logger.recordOutput("EnergyLogger/Power/" + staleKey, 0.0);
        }
      }
      previouslyLoggedRollups.clear();
      previouslyLoggedRollups.addAll(currentRollups.keySet());
    }

    Logger.recordOutput("EnergyLogger/Current/TotalReportedAmps", totalReportedCurrentAmps);
    Logger.recordOutput("EnergyLogger/Power/TotalReportedWatts", totalReportedPowerWatts);
    Logger.recordOutput("EnergyLogger/Energy/TotalReportedJoules", totalReportedEnergyJoules);
    Logger.recordOutput("EnergyLogger/VoltageVolts", voltageVolts);

    if (loopCounter % PDH_LOG_PERIOD_LOOPS == 0) {
      logPowerDistribution(totalReportedPowerWatts);
    }
    currentReports.clear();
    loopCounter++;
  }

  private void report(String key, double... currentAmps) {
    if (key == null) {
      return;
    }

    String trimmedKey = key.strip();
    if (trimmedKey.isEmpty()) {
      return;
    }

    double totalCurrentAmps = 0.0;
    for (double currentAmp : currentAmps) {
      totalCurrentAmps += sanitize(currentAmp);
    }
    currentReports.merge(trimmedKey, totalCurrentAmps, Double::sum);
  }

  private double getBatteryVoltage() {
    return sanitize(RobotController.getBatteryVoltage());
  }

  private void logPowerDistribution(double totalReportedPowerWatts) {
    if (powerDistribution == null) {
      return;
    }

    double pdhPowerWatts = sanitize(powerDistribution.getTotalPower());
    Logger.recordOutput("PowerDistribution/VoltageVolts", sanitize(powerDistribution.getVoltage()));
    Logger.recordOutput(
        "PowerDistribution/TotalCurrentAmps", sanitize(powerDistribution.getTotalCurrent()));
    Logger.recordOutput("PowerDistribution/TotalPowerWatts", pdhPowerWatts);
    Logger.recordOutput(
        "PowerDistribution/TotalEnergyJoules", sanitize(powerDistribution.getTotalEnergy()));
    Logger.recordOutput(
        "PowerDistribution/ChannelCurrentsAmps", sanitize(powerDistribution.getAllCurrents()));
    Logger.recordOutput(
        "EnergyLogger/Power/PdhMinusReportedWatts", pdhPowerWatts - totalReportedPowerWatts);
  }

  private void addRollups(Map<String, Double> rollups, String key, double currentAmps) {
    for (String rollupKey : rollupKeysByReportKey.computeIfAbsent(key, BatteryLogger::rollupKeys)) {
      rollups.merge(rollupKey, currentAmps, Double::sum);
    }
  }

  private static String[] rollupKeys(String key) {
    String[] parts = key.split("/");
    String[] rollupKeys = new String[parts.length];
    StringBuilder prefix = new StringBuilder();
    int count = 0;
    for (String part : parts) {
      if (part.isBlank()) {
        continue;
      }
      if (prefix.length() > 0) {
        prefix.append('/');
      }
      prefix.append(part);
      rollupKeys[count++] = prefix.toString();
    }

    if (count == rollupKeys.length) {
      return rollupKeys;
    }
    String[] compactRollupKeys = new String[count];
    System.arraycopy(rollupKeys, 0, compactRollupKeys, 0, count);
    return compactRollupKeys;
  }

  private static double sanitize(double value) {
    return Double.isFinite(value) && value > 0.0 ? value : 0.0;
  }

  private static double[] sanitize(double[] values) {
    double[] sanitized = new double[values.length];
    for (int i = 0; i < values.length; i++) {
      sanitized[i] = sanitize(values[i]);
    }
    return sanitized;
  }
}
