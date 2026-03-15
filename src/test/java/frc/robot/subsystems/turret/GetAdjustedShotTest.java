package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class GetAdjustedShotTest {
  private final GetAdjustedShot shotCalculator = GetAdjustedShot.getInstance();

  @BeforeEach
  void resetSharedState() {
    shotCalculator.clearShootingParameters();
  }

  @Test
  void hubValidityRecalculatesWhenDistanceChanges() {
    var stationary = new ChassisSpeeds();

    Pose2d inRangePose =
        new Pose2d(
            Constants.FieldConstants.hubTranslation.plus(new Translation2d(2.0, 0.0)),
            Rotation2d.kZero);
    Pose2d outOfRangePose =
        new Pose2d(
            Constants.FieldConstants.hubTranslation.plus(new Translation2d(6.0, 0.0)),
            Rotation2d.kZero);

    assertTrue(
        shotCalculator
            .getParameters(
                inRangePose, stationary, Constants.SubsystemConstants.Turret.robotToTurret1)
            .isValid());
    assertFalse(
        shotCalculator
            .getParameters(
                outOfRangePose, stationary, Constants.SubsystemConstants.Turret.robotToTurret1)
            .isValid());
  }
}
