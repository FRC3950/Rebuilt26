package frc.robot.sim;

import static frc.robot.Constants.SimConstants.Fuel.EFFECTIVE_FLYWHEEL_RADIUS_METERS;
import static frc.robot.Constants.SimConstants.Fuel.TURRET_LAUNCH_HEIGHT_METERS;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

class FuelLaunchCalculatorTest {
  private static final double EPSILON = 1e-9;

  @Test
  void turretLaunchUsesOffsetAndAddsRobotVelocity() {
    FuelLaunchCalculator calculator =
        new FuelLaunchCalculator(FuelLaunchCalculator.LaunchCalibration.identity());
    Pose2d robotPose = new Pose2d(1.0, 2.0, Rotation2d.kCCW_90deg);
    ChassisSpeeds fieldSpeeds = new ChassisSpeeds(1.0, 0.5, 0.0);
    Translation2d robotToTurret = new Translation2d(0.2, -0.1);
    double flywheelRps = 10.0;
    double expectedExitSpeed = 2.0 * Math.PI * EFFECTIVE_FLYWHEEL_RADIUS_METERS * flywheelRps;

    FuelLaunchCalculator.LaunchState launch =
        calculator.calculateTurretLaunch(
            robotPose, fieldSpeeds, robotToTurret, 0.0, 0.0, flywheelRps);

    assertEquals(1.1, launch.position().getX(), EPSILON);
    assertEquals(2.2, launch.position().getY(), EPSILON);
    assertEquals(TURRET_LAUNCH_HEIGHT_METERS, launch.position().getZ(), EPSILON);
    assertEquals(1.0, launch.velocity().getX(), EPSILON);
    assertEquals(0.5 + expectedExitSpeed, launch.velocity().getY(), EPSILON);
    assertEquals(0.0, launch.velocity().getZ(), EPSILON);
  }
}
