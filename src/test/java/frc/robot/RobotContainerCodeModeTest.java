package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class RobotContainerCodeModeTest {
  @Test
  void appliesWhenDisabledAndModeChanges() {
    assertTrue(
        RobotContainer.shouldApplyCodeMode(
            RobotContainer.CodeMode.TUNE, RobotContainer.CodeMode.COMPETITION, true));
  }

  @Test
  void doesNotApplyWhenEnabled() {
    assertFalse(
        RobotContainer.shouldApplyCodeMode(
            RobotContainer.CodeMode.TUNE, RobotContainer.CodeMode.COMPETITION, false));
  }

  @Test
  void doesNotApplyWhenAlreadyActive() {
    assertFalse(
        RobotContainer.shouldApplyCodeMode(
            RobotContainer.CodeMode.COMPETITION, RobotContainer.CodeMode.COMPETITION, true));
  }
}
