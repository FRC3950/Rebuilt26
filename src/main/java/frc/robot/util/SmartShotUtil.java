package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class SmartShotUtil {
  private static final String LOG_ROOT = "SmartShot/";

  private final Timer timer = new Timer();
  private boolean autoMode;
  private boolean autoOverride;
  private boolean warnedMissingAutoWinnerData;
  private boolean wasAutonomousEnabled;
  private boolean wasTeleopEnabled;

  public enum FieldState {
    AUTO(0.0, 20.0),
    TRANSITION(0.0, 10.0),
    SHIFT_1(10.0, 35.0),
    SHIFT_2(35.0, 60.0),
    SHIFT_3(60.0, 85.0),
    SHIFT_4(85.0, 110.0),
    ENDGAME(110.0, 140.0);

    private final double startTime;
    private final double endTime;

    FieldState(double startTime, double endTime) {
      this.startTime = startTime;
      this.endTime = endTime;
    }

    public boolean isActive(double timeSeconds) {
      return startTime <= timeSeconds && timeSeconds < endTime;
    }

    public double timeLeft(double timeSeconds) {
      return Math.max(0.0, endTime - timeSeconds);
    }

    public double timeElapsed(double timeSeconds) {
      return Math.max(0.0, timeSeconds - startTime);
    }
  }

  public SmartShotUtil(boolean autoMode) {
    this.autoMode = autoMode;
    timer.start();
  }

  public SmartShotUtil() {
    this(true);
  }

  public void restartTimer(boolean autoMode) {
    timer.restart();
    this.autoMode = autoMode;
  }

  public void publish() {
    syncModeWithDriverStation();

    FieldState fieldState = getCurrentFieldState();
    boolean activeInCurrentShift = isActiveShift();
    boolean didWinAuto = didWinAuto();
    boolean hasAutoWinnerData = hasAutoWinnerData();
    double matchTimeRemainingSeconds = getMatchTimeRemainingSeconds();
    double timeUntilShiftBoundarySeconds = getTimeUntilShiftBoundarySeconds();
    double timeUntilOurShiftStartsSeconds = getTimeUntilOurShiftStartsSeconds();
    double timeUntilOurShiftEndsSeconds = getTimeUntilOurShiftEndsSeconds();

    Logger.recordOutput(LOG_ROOT + "FieldState", fieldState.name());
    Logger.recordOutput(LOG_ROOT + "MatchTimeRemainingSec", matchTimeRemainingSeconds);
    Logger.recordOutput(LOG_ROOT + "ActiveInCurrentShift", activeInCurrentShift);
    Logger.recordOutput(LOG_ROOT + "TimeUntilShiftBoundarySec", timeUntilShiftBoundarySeconds);
    Logger.recordOutput(LOG_ROOT + "TimeUntilOurShiftStartsSec", timeUntilOurShiftStartsSeconds);
    Logger.recordOutput(LOG_ROOT + "TimeUntilOurShiftEndsSec", timeUntilOurShiftEndsSeconds);
    Logger.recordOutput(LOG_ROOT + "DidWinAuto", didWinAuto);
    Logger.recordOutput(LOG_ROOT + "HasAutoWinnerData", hasAutoWinnerData);

    SmartDashboard.putString(LOG_ROOT + "FieldState", fieldState.name());
    SmartDashboard.putNumber(LOG_ROOT + "MatchTimeRemainingSec", matchTimeRemainingSeconds);
    SmartDashboard.putBoolean(LOG_ROOT + "ActiveInCurrentShift", activeInCurrentShift);
    SmartDashboard.putNumber(LOG_ROOT + "TimeUntilShiftBoundarySec", timeUntilShiftBoundarySeconds);
    SmartDashboard.putNumber(LOG_ROOT + "TimeUntilOurShiftStartsSec", timeUntilOurShiftStartsSeconds);
    SmartDashboard.putNumber(LOG_ROOT + "TimeUntilOurShiftEndsSec", timeUntilOurShiftEndsSeconds);
    SmartDashboard.putBoolean(LOG_ROOT + "DidWinAuto", didWinAuto);
    SmartDashboard.putBoolean(LOG_ROOT + "HasAutoWinnerData", hasAutoWinnerData);
  }

  public FieldState getCurrentFieldState() {
    if (autoMode) {
      return FieldState.AUTO;
    }

    double timeSeconds = timer.get();

    for (FieldState state : FieldState.values()) {
      if (state != FieldState.AUTO && state.isActive(timeSeconds)) {
        return state;
      }
    }

    return FieldState.ENDGAME;
  }

  public boolean isActiveShift() {
    boolean wonAuto = didWinAuto();

    return switch (getCurrentFieldState()) {
      case AUTO, TRANSITION, ENDGAME -> true;
      case SHIFT_1, SHIFT_3 -> wonAuto;
      case SHIFT_2, SHIFT_4 -> !wonAuto;
    };
  }

  public void overrideFMSAutoVictor(boolean didWin) {
    autoOverride = didWin;
  }

  public boolean hasAutoWinnerData() {
    String winner = DriverStation.getGameSpecificMessage();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    return winner != null && !winner.isBlank() && alliance.isPresent();
  }

  public boolean didWinAuto() {
    if (!hasAutoWinnerData()) {
      if (DriverStation.isFMSAttached() && !warnedMissingAutoWinnerData) {
        DriverStation.reportWarning("No FMS auto winner data available", false);
        warnedMissingAutoWinnerData = true;
      }
      return autoOverride;
    }

    warnedMissingAutoWinnerData = false;

    String allianceLetter =
        DriverStation.getAlliance().orElseThrow() == Alliance.Blue ? "B" : "R";

    return allianceLetter.equalsIgnoreCase(DriverStation.getGameSpecificMessage().trim());
  }

  public double getMatchTimeRemainingSeconds() {
    double matchTimeSeconds = DriverStation.getMatchTime();
    if (matchTimeSeconds >= 0.0) {
      return matchTimeSeconds;
    }

    return autoMode ? FieldState.AUTO.timeLeft(timer.get()) : FieldState.ENDGAME.timeLeft(timer.get());
  }

  public double getTimeUntilShiftBoundarySeconds() {
    return getCurrentFieldState().timeLeft(timer.get());
  }

  public double getTimeUntilOurShiftStartsSeconds() {
    return isActiveShift() ? 0.0 : getTimeUntilShiftBoundarySeconds();
  }

  public double getTimeUntilOurShiftEndsSeconds() {
    return isActiveShift() ? getTimeUntilShiftBoundarySeconds() : 0.0;
  }

  private void syncModeWithDriverStation() {
    boolean autonomousEnabled = DriverStation.isAutonomousEnabled();
    boolean teleopEnabled = DriverStation.isTeleopEnabled();

    if (autonomousEnabled && !wasAutonomousEnabled) {
      restartTimer(true);
    } else if (teleopEnabled && !wasTeleopEnabled) {
      restartTimer(false);
    }

    wasAutonomousEnabled = autonomousEnabled;
    wasTeleopEnabled = teleopEnabled;
  }
}
