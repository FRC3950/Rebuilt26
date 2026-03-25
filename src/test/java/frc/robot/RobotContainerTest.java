package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class RobotContainerTest {
  @Test
  void distanceCommandStartsOnlyOnRisingToggleEdge() {
    assertTrue(RobotContainer.shouldStartDistanceCommand(false, true));
    assertFalse(RobotContainer.shouldStartDistanceCommand(false, false));
    assertFalse(RobotContainer.shouldStartDistanceCommand(true, true));
    assertFalse(RobotContainer.shouldStartDistanceCommand(true, false));
  }
}
