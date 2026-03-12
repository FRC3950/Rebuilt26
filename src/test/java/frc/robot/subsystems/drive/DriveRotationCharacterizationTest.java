package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

public class DriveRotationCharacterizationTest {
  @Test
  void rotatesModuleHeadingToTangent() {
    Rotation2d heading = Drive.getRotationCharacterizationHeading(new Translation2d(1.0, 0.0));

    assertEquals(90.0, heading.getDegrees(), 1e-9);
  }
}
