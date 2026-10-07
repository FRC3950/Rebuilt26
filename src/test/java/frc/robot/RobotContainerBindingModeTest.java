package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class RobotContainerBindingModeTest {
  @Test
  void appliesWhenDisabledAndModeChanges() {
    assertTrue(
        RobotContainer.shouldApplyBindingMode(
            RobotContainer.BindingMode.TUNE, RobotContainer.BindingMode.DEFAULT, true));
  }

  @Test
  void doesNotApplyWhenEnabled() {
    assertFalse(
        RobotContainer.shouldApplyBindingMode(
            RobotContainer.BindingMode.TUNE, RobotContainer.BindingMode.DEFAULT, false));
  }

  @Test
  void doesNotApplyWhenAlreadyActive() {
    assertFalse(
        RobotContainer.shouldApplyBindingMode(
            RobotContainer.BindingMode.DEFAULT, RobotContainer.BindingMode.DEFAULT, true));
  }
}
