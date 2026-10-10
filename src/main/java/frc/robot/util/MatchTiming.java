package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Publishes the teleop phase and seconds until its end using the Driver Station clock. */
public final class MatchTiming extends SubsystemBase {
  private static final String PHASE_KEY = "Current Phase";
  private static final String COUNTDOWN_KEY = "Shift Countdown";
  // Remaining teleop seconds at each phase's end, as used in battlecry-gotham.
  private static final double[] PHASE_END_TIMES_SEC = {130.0, 105.0, 80.0, 55.0, 30.0, 0.0};
  private static final String[] PHASE_NAMES = {
    "Transition", "Shift 1", "Shift 2", "Shift 3", "Shift 4", "Endgame"
  };

  public MatchTiming() {
    publish("Unknown", 0.0);
  }

  @Override
  public void periodic() {
    double matchTimeSec = DriverStation.getMatchTime();
    // An unavailable DS clock must not appear as an endgame countdown.
    if (!DriverStation.isTeleopEnabled() || !Double.isFinite(matchTimeSec) || matchTimeSec < 0.0) {
      publish("Unknown", 0.0);
      return;
    }

    for (int phase = 0; phase < PHASE_END_TIMES_SEC.length; phase++) {
      if (matchTimeSec > PHASE_END_TIMES_SEC[phase] || phase == PHASE_END_TIMES_SEC.length - 1) {
        publish(PHASE_NAMES[phase], matchTimeSec - PHASE_END_TIMES_SEC[phase]);
        return;
      }
    }
  }

  private static void publish(String phaseName, double countdownSec) {
    SmartDashboard.putString(PHASE_KEY, phaseName);
    SmartDashboard.putNumber(COUNTDOWN_KEY, countdownSec);
  }
}
