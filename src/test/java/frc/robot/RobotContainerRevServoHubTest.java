package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class RobotContainerRevServoHubTest {
  @Test
  void revServoHubCanId63IsInvalid() {
    assertTrue(RobotContainer.isValidRevServoHubCanId(0));
    assertTrue(RobotContainer.isValidRevServoHubCanId(62));
    assertFalse(RobotContainer.isValidRevServoHubCanId(63));
  }
}
