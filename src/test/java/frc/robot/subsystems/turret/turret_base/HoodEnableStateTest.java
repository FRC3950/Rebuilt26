package frc.robot.subsystems.turret.turret_base;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.REVLibError;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class HoodEnableStateTest {
  @Test
  void successfulStateIsWrittenOnceRegardlessOfRepeatedUpdates() {
    List<Boolean> writes = new ArrayList<>();
    var state =
        new Hood.EnableState(
            enabled -> {
              writes.add(enabled);
              return REVLibError.kOk;
            });

    for (int tick = 0; tick < 1000; tick++) {
      state.setEnabled(true);
    }
    assertEquals(List.of(true), writes);
  }

  @Test
  void stopAndResumeWriteStateChangesAndRepeatedStopsAreSkipped() {
    List<Boolean> writes = new ArrayList<>();
    var state =
        new Hood.EnableState(
            enabled -> {
              writes.add(enabled);
              return REVLibError.kOk;
            });

    state.setEnabled(true);
    for (int tick = 0; tick < 1000; tick++) {
      state.setEnabled(false);
    }
    state.setEnabled(true);
    assertEquals(List.of(true, false, true), writes);
  }

  @Test
  void failedWriteIsRetriedOnTheNextCallAndSuccessIsCached() {
    List<Boolean> writes = new ArrayList<>();
    var state =
        new Hood.EnableState(
            enabled -> {
              writes.add(enabled);
              return writes.size() == 1 ? REVLibError.kError : REVLibError.kOk;
            });

    state.setEnabled(true);
    assertEquals(List.of(true), writes);

    state.setEnabled(true);
    state.setEnabled(true);
    assertEquals(List.of(true, true), writes);
  }

  @Test
  void failedStateChangeInvalidatesThePreviouslySuccessfulState() {
    List<Boolean> writes = new ArrayList<>();
    var state =
        new Hood.EnableState(
            enabled -> {
              writes.add(enabled);
              return enabled ? REVLibError.kOk : REVLibError.kCANDisconnected;
            });

    state.setEnabled(true);
    state.setEnabled(false);
    state.setEnabled(true);
    assertEquals(List.of(true, false, true), writes);
  }
}
