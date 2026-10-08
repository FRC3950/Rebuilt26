package frc.robot;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.Test;

public class RobotContainerBindingModeTest {
  @Test
  void defaultIsTheOnlyAvailableMode() {
    assertArrayEquals(
        new RobotContainer.BindingMode[] {RobotContainer.BindingMode.DEFAULT},
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
}
