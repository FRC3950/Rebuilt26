package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class RobotContainerBindingModeTest {
  @Test
  void appliesWhenDisabledAndModeChanges() {
    assertTrue(
        RobotContainer.shouldApplyBindingMode(
            RobotContainer.BindingMode.CRAZY, RobotContainer.BindingMode.COMPETITION, true));
  }

  @Test
  void doesNotApplyWhenEnabled() {
    assertFalse(
        RobotContainer.shouldApplyBindingMode(
            RobotContainer.BindingMode.CRAZY, RobotContainer.BindingMode.COMPETITION, false));
  }

  @Test
  void doesNotApplyWhenAlreadyActive() {
    assertFalse(
        RobotContainer.shouldApplyBindingMode(
            RobotContainer.BindingMode.COMPETITION,
            RobotContainer.BindingMode.COMPETITION,
            true));
  }
}
