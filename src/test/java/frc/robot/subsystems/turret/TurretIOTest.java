package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.turret.turret_base.azimuth.Azimuth;
import frc.robot.subsystems.turret.turret_base.azimuth.AzimuthIO;
import frc.robot.subsystems.turret.turret_base.flywheels.Flywheels;
import frc.robot.subsystems.turret.turret_base.flywheels.FlywheelsIO;
import frc.robot.subsystems.turret.turret_base.hood.Hood;
import frc.robot.subsystems.turret.turret_base.hood.HoodIO;
import org.junit.jupiter.api.Test;

class TurretIOTest {
  private static class FakeAzimuthIO implements AzimuthIO {
    double requestedAngleDeg = 0.0;
    double requestedVelocityDegPerSec = 0.0;

    @Override
    public void updateInputs(AzimuthIOInputs inputs) {
      inputs.positionDeg = requestedAngleDeg;
    }

    @Override
    public void setTargetAngleDeg(double targetAngleDeg) {
      requestedAngleDeg = targetAngleDeg;
    }

    @Override
    public void setTargetAngleDeg(double targetAngleDeg, double targetVelocityDegPerSec) {
      requestedAngleDeg = targetAngleDeg;
      requestedVelocityDegPerSec = targetVelocityDegPerSec;
    }
  }

  private static class FakeHoodIO implements HoodIO {
    double requestedAngleDeg = 0.0;

    @Override
    public void updateInputs(HoodIOInputs inputs) {
      inputs.positionDeg = requestedAngleDeg;
    }

    @Override
    public void setAngleDeg(double hoodAngleDeg) {
      requestedAngleDeg = hoodAngleDeg;
    }
  }

  private static class FakeFlywheelsIO implements FlywheelsIO {
    double requestedRps = 0.0;
    double leaderVelocityRps = 0.0;
    double followerVelocityRps = 0.0;

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
      inputs.leaderVelocityRps = leaderVelocityRps;
      inputs.followerVelocityRps = followerVelocityRps;
    }

    @Override
    public void setTargetRps(double flywheelSpeedRps) {
      requestedRps = flywheelSpeedRps;
    }
  }

  @Test
  void requestedHoodAngleUsesServoSetpoint() {
    Turret turret = createTestTurret(new FakeAzimuthIO());

    turret.runSetpoints(Rotation2d.kZero, 45.0, 60.0);

    assertEquals(
        frc.robot.Constants.SubsystemConstants.Turret.maxHoodAngle,
        turret.getRequestedHoodAngleDeg());
  }

  @Test
  void zeroAzimuthLockUsesDegrees() {
    FakeAzimuthIO azimuthIO = new FakeAzimuthIO();
    Turret turret = createTestTurret(azimuthIO, -380.0, 1.0);

    turret.runZeroAzimuthTarget(
        new GetAdjustedShot.ShootingParameters(true, Rotation2d.kZero, 0.0, 20.0, 60.0, ""));

    assertEquals(-135.0, azimuthIO.requestedAngleDeg, 1e-9);
  }

  @Test
  void autoTargetPassesVelocitySetpointToAzimuth() {
    FakeAzimuthIO azimuthIO = new FakeAzimuthIO();
    Turret turret = createTestTurret(azimuthIO);
    double turretVelocityRadPerSec = 1.5;

    turret.runAutoTarget(
        new GetAdjustedShot.ShootingParameters(
            true, Rotation2d.fromDegrees(10.0), turretVelocityRadPerSec, 20.0, 60.0, ""));

    assertEquals(
        Units.radiansToDegrees(turretVelocityRadPerSec),
        azimuthIO.requestedVelocityDegPerSec,
        1e-9);
    assertEquals(
        10.0 + Units.radiansToDegrees(turretVelocityRadPerSec) * 0.02,
        azimuthIO.requestedAngleDeg,
        1e-9);
  }

  @Test
  void autoTargetUsesSharedDashboardFlywheelScale() {
    FakeFlywheelsIO leftFlywheelsIO = new FakeFlywheelsIO();
    FakeFlywheelsIO rightFlywheelsIO = new FakeFlywheelsIO();
    Turret leftTurret = createTestTurret(new FakeAzimuthIO(), leftFlywheelsIO);
    Turret rightTurret = createTestTurret(new FakeAzimuthIO(), rightFlywheelsIO);
    SmartDashboard.putNumber(Turret.FLYWHEEL_SCALE_DASHBOARD_KEY, 1.08);
    var shotParameters =
        new GetAdjustedShot.ShootingParameters(
            true, Rotation2d.fromDegrees(10.0), 0.0, 20.0, 60.0, "");

    leftTurret.runAutoTarget(shotParameters);
    rightTurret.runAutoTarget(shotParameters);

    assertEquals(64.8, leftFlywheelsIO.requestedRps, 1e-9);
    assertEquals(64.8, rightFlywheelsIO.requestedRps, 1e-9);
  }

  @Test
  void flywheelReadyAllowsTwoPointFiveRpsErrorOnBothMotors() {
    FakeFlywheelsIO flywheelsIO = new FakeFlywheelsIO();
    Turret turret = createTestTurret(new FakeAzimuthIO(), flywheelsIO);

    turret.runSetpoints(Rotation2d.kZero, 20.0, 60.0);
    flywheelsIO.leaderVelocityRps = 57.5;
    flywheelsIO.followerVelocityRps = -62.5;
    turret.periodic();

    assertTrue(turret.isFlywheelReadyForFeed());

    flywheelsIO.leaderVelocityRps = 57.49;
    flywheelsIO.followerVelocityRps = -60.0;
    turret.periodic();

    assertFalse(turret.isFlywheelReadyForFeed());
  }

  @Test
  void simVisualizationUsesCommandedAzimuth() {
    Turret turret = createTestTurret(new FakeAzimuthIO());

    turret.runSetpoints(Rotation2d.fromDegrees(42.0), 20.0, 60.0);

    assertEquals(
        42.0,
        Units.radiansToDegrees(turret.getRobotPose3d(Translation2d.kZero).getRotation().getZ()),
        1e-9);
  }

  @Test
  void turretLogKeyIsSideSpecific() {
    Turret turret = createTestTurret(new FakeAzimuthIO());

    assertEquals("Turret/Test", turret.getLogKey());
  }

  private static Turret createTestTurret(FakeAzimuthIO azimuthIO) {
    return createTestTurret(azimuthIO, -180.0, 180.0);
  }

  private static Turret createTestTurret(FakeAzimuthIO azimuthIO, FakeFlywheelsIO flywheelsIO) {
    return createTestTurret(azimuthIO, flywheelsIO, -180.0, 180.0);
  }

  private static Turret createTestTurret(
      FakeAzimuthIO azimuthIO, double minAzimuthDeg, double maxAzimuthDeg) {
    return createTestTurret(azimuthIO, new FakeFlywheelsIO(), minAzimuthDeg, maxAzimuthDeg);
  }

  private static Turret createTestTurret(
      FakeAzimuthIO azimuthIO,
      FakeFlywheelsIO flywheelsIO,
      double minAzimuthDeg,
      double maxAzimuthDeg) {
    return new Turret(
        "TestTurret",
        new Azimuth("Test/Azimuth", azimuthIO),
        new Hood("Test/Hood", new FakeHoodIO()),
        new Flywheels("Test/Flywheels", flywheelsIO),
        minAzimuthDeg,
        maxAzimuthDeg);
  }
}
