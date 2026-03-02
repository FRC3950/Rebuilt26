package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.turret.GetAdjustedShot;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class HubShiftUtil {
  public enum AutoWinnerMode {
    AUTO,
    RED,
    BLUE
  }

  public enum ShiftEnum {
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME,
    AUTO,
    DISABLED
  }

  public record ShiftInfo(
      ShiftEnum currentShift, double elapsedTimeSec, double remainingTimeSec, boolean active) {}

  public static final double AUTO_DURATION_SEC = 20.0;
  public static final double TELEOP_DURATION_SEC = 140.0;
  private static final double GAME_DATA_TIMEOUT_SEC = 4.5;

  private static final double[] TELEOP_SHIFT_START_TIMES_SEC = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
  private static final double[] TELEOP_SHIFT_END_TIMES_SEC = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};
  private static final ShiftEnum[] TELEOP_SHIFT_ENUMS = {
    ShiftEnum.TRANSITION,
    ShiftEnum.SHIFT1,
    ShiftEnum.SHIFT2,
    ShiftEnum.SHIFT3,
    ShiftEnum.SHIFT4,
    ShiftEnum.ENDGAME
  };

  private static final boolean[] WINNING_ALLIANCE_SCHEDULE = {true, false, true, false, true, true};
  private static final boolean[] LOSING_ALLIANCE_SCHEDULE = {true, true, false, true, false, true};
  private static final boolean[] FAIL_OPEN_SCHEDULE = {true, true, true, true, true, true};

  private static final double MIN_FUEL_COUNT_DELAY_SEC = 1.0;
  private static final double MAX_FUEL_COUNT_DELAY_SEC = 2.0;
  private static final double SHIFT_END_FUEL_COUNT_EXTENSION_SEC = 3.0;

  private static final double APPROACHING_ACTIVE_FUDGE_SEC =
      -(GetAdjustedShot.getMinTimeOfFlight() + MIN_FUEL_COUNT_DELAY_SEC);
  private static final double ENDING_ACTIVE_FUDGE_SEC =
      SHIFT_END_FUEL_COUNT_EXTENSION_SEC
          - (GetAdjustedShot.getMaxTimeOfFlight() + MAX_FUEL_COUNT_DELAY_SEC);

  private static final String KEY_MODE = "Smart Shot/Auto Winner Mode";
  private static final String KEY_FMS_FROZEN = "Smart Shot/Game Data Frozen";
  private static final String KEY_FMS_WINNER = "Smart Shot/FMS Winner";
  private static final String KEY_WINNER_RESOLVED = "Smart Shot/Winner Resolved";

  private static final Timer matchTimer = new Timer();
  private static boolean matchTimerStarted = false;
  private static double teleopStartSec = Double.NaN;

  private static boolean fmsLookupFrozen = false;
  private static boolean missingGameDataWarningSent = false;
  private static Optional<Alliance> latchedFmsWinnerAlliance = Optional.empty();

  private static Supplier<AutoWinnerMode> autoWinnerModeSupplier = () -> AutoWinnerMode.AUTO;

  private static final Elastic.Notification missingGameDataWarningNotification =
      new Elastic.Notification()
          .withLevel(Elastic.NotificationLevel.WARNING)
          .withTitle("Auto winner data missing")
          .withDescription(
              "FMS game data was not received by teleop +4.5s. Smart Shot is using fail-open mode.")
          .withDisplaySeconds(6.0);

  private HubShiftUtil() {}

  public static void initializeMatchTimer() {
    matchTimer.restart();
    matchTimerStarted = true;
    teleopStartSec = Double.NaN;
    fmsLookupFrozen = false;
    missingGameDataWarningSent = false;
    latchedFmsWinnerAlliance = Optional.empty();
  }

  public static void ensureMatchTimerStarted() {
    if (!matchTimerStarted) {
      initializeMatchTimer();
    }
  }

  public static void markTeleopStart() {
    ensureMatchTimerStarted();
    if (Double.isNaN(teleopStartSec)) {
      teleopStartSec = matchTimer.get();
    }
  }

  public static void setAutoWinnerModeSupplier(Supplier<AutoWinnerMode> supplier) {
    autoWinnerModeSupplier = supplier != null ? supplier : (() -> AutoWinnerMode.AUTO);
  }

  public static void periodic() {
    AutoWinnerMode mode = getAutoWinnerMode();
    Logger.recordOutput(KEY_MODE, mode.name());
    Logger.recordOutput(KEY_FMS_FROZEN, fmsLookupFrozen);

    if (DriverStation.isTeleopEnabled()) {
      markTeleopStart();
      if (mode == AutoWinnerMode.AUTO && !fmsLookupFrozen) {
        Optional<Alliance> parsedWinner = parseAutoWinnerFromFms();
        if (parsedWinner.isPresent()) {
          latchedFmsWinnerAlliance = parsedWinner;
        } else if (getTeleopElapsedTimeSec() >= GAME_DATA_TIMEOUT_SEC) {
          fmsLookupFrozen = true;
          if (latchedFmsWinnerAlliance.isEmpty()) {
            sendMissingGameDataWarning();
          }
        }
      }
    }

    Optional<Alliance> winner = getAutoWinnerAlliance();
    Logger.recordOutput(KEY_WINNER_RESOLVED, winner.isPresent());
    Logger.recordOutput(KEY_FMS_WINNER, winner.map(Enum::name).orElse("Unknown"));
  }

  public static ShiftInfo getOfficialShiftInfo() {
    return getShiftInfo(
        getCurrentSchedule(), TELEOP_SHIFT_START_TIMES_SEC, TELEOP_SHIFT_END_TIMES_SEC, 0.0);
  }

  public static ShiftInfo getShiftedShiftInfo() {
    return getShiftedShiftInfo(0.0);
  }

  public static boolean isHubActiveAtPredictedImpact(double predictionOffsetSec) {
    return getShiftedShiftInfo(predictionOffsetSec).active();
  }

  private static ShiftInfo getShiftedShiftInfo(double predictionOffsetSec) {
    boolean[] schedule = getCurrentSchedule();

    if (schedule[1]) {
      double[] shiftedStartTimes = {
        0.0,
        10.0,
        35.0 + ENDING_ACTIVE_FUDGE_SEC,
        60.0 + APPROACHING_ACTIVE_FUDGE_SEC,
        85.0 + ENDING_ACTIVE_FUDGE_SEC,
        110.0 + APPROACHING_ACTIVE_FUDGE_SEC
      };
      double[] shiftedEndTimes = {
        10.0,
        35.0 + ENDING_ACTIVE_FUDGE_SEC,
        60.0 + APPROACHING_ACTIVE_FUDGE_SEC,
        85.0 + ENDING_ACTIVE_FUDGE_SEC,
        110.0 + APPROACHING_ACTIVE_FUDGE_SEC,
        140.0
      };
      return getShiftInfo(schedule, shiftedStartTimes, shiftedEndTimes, predictionOffsetSec);
    }

    double[] shiftedStartTimes = {
      0.0,
      10.0 + ENDING_ACTIVE_FUDGE_SEC,
      35.0 + APPROACHING_ACTIVE_FUDGE_SEC,
      60.0 + ENDING_ACTIVE_FUDGE_SEC,
      85.0 + APPROACHING_ACTIVE_FUDGE_SEC,
      110.0
    };
    double[] shiftedEndTimes = {
      10.0 + ENDING_ACTIVE_FUDGE_SEC,
      35.0 + APPROACHING_ACTIVE_FUDGE_SEC,
      60.0 + ENDING_ACTIVE_FUDGE_SEC,
      85.0 + APPROACHING_ACTIVE_FUDGE_SEC,
      110.0,
      140.0
    };
    return getShiftInfo(schedule, shiftedStartTimes, shiftedEndTimes, predictionOffsetSec);
  }

  private static ShiftInfo getShiftInfo(
      boolean[] schedule,
      double[] shiftStartTimesSec,
      double[] shiftEndTimesSec,
      double predictionOffsetSec) {
    ensureMatchTimerStarted();

    double currentMatchTimeSec = matchTimer.get();
    if (DriverStation.isAutonomousEnabled()) {
      double stateElapsedTimeSec = Math.max(0.0, currentMatchTimeSec + predictionOffsetSec);
      double stateRemainingTimeSec = Math.max(0.0, AUTO_DURATION_SEC - stateElapsedTimeSec);
      return new ShiftInfo(ShiftEnum.AUTO, stateElapsedTimeSec, stateRemainingTimeSec, true);
    }

    if (!DriverStation.isTeleopEnabled()) {
      return new ShiftInfo(ShiftEnum.DISABLED, 0.0, 0.0, false);
    }

    markTeleopStart();

    double teleopElapsedSec = getTeleopElapsedTimeSec() + predictionOffsetSec;
    double clampedTeleopElapsedSec = Math.max(0.0, Math.min(TELEOP_DURATION_SEC, teleopElapsedSec));

    int shiftIndex = shiftStartTimesSec.length - 1;
    for (int i = 0; i < shiftStartTimesSec.length; i++) {
      if (clampedTeleopElapsedSec >= shiftStartTimesSec[i]
          && clampedTeleopElapsedSec < shiftEndTimesSec[i]) {
        shiftIndex = i;
        break;
      }
    }

    double elapsedTimeSec = Math.max(0.0, clampedTeleopElapsedSec - shiftStartTimesSec[shiftIndex]);
    double remainingTimeSec = Math.max(0.0, shiftEndTimesSec[shiftIndex] - clampedTeleopElapsedSec);

    if (shiftIndex > 0 && schedule[shiftIndex] == schedule[shiftIndex - 1]) {
      elapsedTimeSec = Math.max(0.0, clampedTeleopElapsedSec - shiftStartTimesSec[shiftIndex - 1]);
    }

    if (shiftIndex < shiftEndTimesSec.length - 1
        && schedule[shiftIndex] == schedule[shiftIndex + 1]) {
      remainingTimeSec = Math.max(0.0, shiftEndTimesSec[shiftIndex + 1] - clampedTeleopElapsedSec);
    }

    return new ShiftInfo(
        TELEOP_SHIFT_ENUMS[shiftIndex], elapsedTimeSec, remainingTimeSec, schedule[shiftIndex]);
  }

  private static AutoWinnerMode getAutoWinnerMode() {
    AutoWinnerMode mode = autoWinnerModeSupplier.get();
    return mode != null ? mode : AutoWinnerMode.AUTO;
  }

  private static boolean[] getCurrentSchedule() {
    Optional<Alliance> winnerAlliance = getAutoWinnerAlliance();
    Optional<Alliance> ourAlliance = DriverStation.getAlliance();

    if (winnerAlliance.isEmpty() || ourAlliance.isEmpty()) {
      return FAIL_OPEN_SCHEDULE;
    }

    return winnerAlliance.get() == ourAlliance.get()
        ? WINNING_ALLIANCE_SCHEDULE
        : LOSING_ALLIANCE_SCHEDULE;
  }

  private static Optional<Alliance> getAutoWinnerAlliance() {
    AutoWinnerMode mode = getAutoWinnerMode();

    if (mode == AutoWinnerMode.RED) {
      return Optional.of(Alliance.Red);
    }
    if (mode == AutoWinnerMode.BLUE) {
      return Optional.of(Alliance.Blue);
    }

    if (!fmsLookupFrozen) {
      Optional<Alliance> parsedWinner = parseAutoWinnerFromFms();
      if (parsedWinner.isPresent()) {
        latchedFmsWinnerAlliance = parsedWinner;
      }
    }

    return latchedFmsWinnerAlliance;
  }

  private static Optional<Alliance> parseAutoWinnerFromFms() {
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData == null || gameData.isEmpty()) {
      return Optional.empty();
    }

    char autoWinner = Character.toUpperCase(gameData.charAt(0));
    if (autoWinner == 'R') {
      return Optional.of(Alliance.Red);
    }
    if (autoWinner == 'B') {
      return Optional.of(Alliance.Blue);
    }
    return Optional.empty();
  }

  private static double getTeleopElapsedTimeSec() {
    if (!matchTimerStarted || Double.isNaN(teleopStartSec)) {
      return 0.0;
    }
    return Math.max(0.0, matchTimer.get() - teleopStartSec);
  }

  private static void sendMissingGameDataWarning() {
    if (missingGameDataWarningSent) {
      return;
    }
    Elastic.sendNotification(missingGameDataWarningNotification);
    missingGameDataWarningSent = true;
  }
}
