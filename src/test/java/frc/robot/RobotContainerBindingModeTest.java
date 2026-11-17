package frc.robot;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class RobotContainerBindingModeTest {
  @Test
  void defaultAndDemoAreTheAvailableModes() {
    assertArrayEquals(
        new RobotContainer.BindingMode[] {
          RobotContainer.BindingMode.DEFAULT, RobotContainer.BindingMode.DEMO
        },
        RobotContainer.BindingMode.values());
  }

  @Test
  void doesNotApplyWhenAlreadyActive() {
    assertFalse(
        RobotContainer.shouldApplyBindingMode(
            RobotContainer.BindingMode.DEFAULT, RobotContainer.BindingMode.DEFAULT, true));
    assertFalse(
        RobotContainer.shouldApplyBindingMode(
            RobotContainer.BindingMode.DEFAULT, RobotContainer.BindingMode.DEFAULT, false));
  }

  @Test
  void switchesToDemoOnlyWhileDisabled() {
    assertTrue(
        RobotContainer.shouldApplyBindingMode(
            RobotContainer.BindingMode.DEMO, RobotContainer.BindingMode.DEFAULT, true));
    assertFalse(
        RobotContainer.shouldApplyBindingMode(
            RobotContainer.BindingMode.DEMO, RobotContainer.BindingMode.DEFAULT, false));
  }
}
