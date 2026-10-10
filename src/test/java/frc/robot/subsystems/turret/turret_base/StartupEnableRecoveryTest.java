package frc.robot.subsystems.turret.turret_base;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class StartupEnableRecoveryTest {
  @Test
  void forgottenButtonAttemptsOnceAndRequiresDisableBeforeRetry() {
    var recovery = new StartupEnableRecovery();
    assertFalse(recovery.shouldAttempt(false, false, false, true));
    assertTrue(recovery.shouldAttempt(true, false, false, true));
    assertFalse(recovery.shouldAttempt(true, false, true, true));
    // A failed verification must not keep rewriting sensor position while enabled.
    assertFalse(recovery.shouldAttempt(true, false, false, true));
    assertFalse(recovery.shouldAttempt(false, false, false, true));
    assertTrue(recovery.shouldAttempt(true, false, false, true));
  }

  @Test
  void successfulManualConfirmationIsPreservedEvenIfFeedbackLaterFails() {
    var recovery = new StartupEnableRecovery();
    assertFalse(recovery.shouldAttempt(false, true, false, true));
    assertFalse(recovery.shouldAttempt(true, true, false, true));
    assertFalse(recovery.shouldAttempt(true, false, false, true));
  }

  @Test
  void enablingDuringManualVerificationDoesNotStartAnotherWrite() {
    var recovery = new StartupEnableRecovery();
    assertFalse(recovery.shouldAttempt(false, false, true, true));
    assertFalse(recovery.shouldAttempt(true, false, true, true));
    assertFalse(recovery.shouldAttempt(true, true, false, true));
  }

  @Test
  void disabledTurretCannotRecoverMidMatchWhenItsSwitchIsCleared() {
    var recovery = new StartupEnableRecovery();
    assertFalse(recovery.shouldAttempt(false, false, false, false));
    assertFalse(recovery.shouldAttempt(true, false, false, false));
    assertFalse(recovery.shouldAttempt(true, false, false, true));
    assertFalse(recovery.shouldAttempt(false, false, false, true));
    assertTrue(recovery.shouldAttempt(true, false, false, true));
  }
}
