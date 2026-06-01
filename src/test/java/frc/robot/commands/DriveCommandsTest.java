package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class DriveCommandsTest {
  private static final double EPSILON = 1e-9;

  @Test
  void forwardDirectionFallbackUsesAllianceRelativeDriveFrame() {
    var speeds =
        DriveCommands.calculateForwardDirectionSpeeds(1.0, 0.0, 0.0, Rotation2d.kPi, null, 1.0);

    assertEquals(-1.0, speeds.vxMetersPerSecond, EPSILON);
    assertEquals(0.0, speeds.vyMetersPerSecond, EPSILON);
    assertEquals(0.0, speeds.omegaRadiansPerSecond, EPSILON);
  }
}
