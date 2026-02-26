package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class TeleopDriveModeTest {
  @Test
  void trenchLockHasHighestPriority() {
    assertEquals(
        TeleopDrive.DriveMode.TRENCH_LOCK, TeleopDrive.selectDriveMode(true, true, true, true));
  }

  @Test
  void snakeWhenEnabledIntakingAndMoving() {
    assertEquals(TeleopDrive.DriveMode.SNAKE, TeleopDrive.selectDriveMode(false, true, true, true));
  }

  @Test
  void normalWhenSnakeDisabled() {
    assertEquals(
        TeleopDrive.DriveMode.NORMAL, TeleopDrive.selectDriveMode(false, false, true, true));
  }

  @Test
  void normalWhenNotIntaking() {
    assertEquals(
        TeleopDrive.DriveMode.NORMAL, TeleopDrive.selectDriveMode(false, true, false, true));
  }

  @Test
  void normalWhenNotMoving() {
    assertEquals(
        TeleopDrive.DriveMode.NORMAL, TeleopDrive.selectDriveMode(false, true, true, false));
  }
}
