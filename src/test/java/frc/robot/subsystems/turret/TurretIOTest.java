package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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

    @Override
    public void updateInputs(AzimuthIOInputs inputs) {
      inputs.positionDeg = requestedAngleDeg;
    }

    @Override
    public void setTargetAngleDeg(double targetAngleDeg) {
      requestedAngleDeg = targetAngleDeg;
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

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
      inputs.leaderVelocityRps = requestedRps;
      inputs.followerVelocityRps = -requestedRps;
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
  void simVisualizationUsesCommandedAzimuth() {
    Turret turret = createTestTurret(new FakeAzimuthIO());

    turret.runSetpoints(Rotation2d.fromDegrees(42.0), 20.0, 60.0);

    assertEquals(
        42.0,
        Units.radiansToDegrees(turret.getRobotPose3d(Translation2d.kZero).getRotation().getZ()),
        1e-9);
  }

  private static Turret createTestTurret(FakeAzimuthIO azimuthIO) {
    return createTestTurret(azimuthIO, -180.0, 180.0);
  }

  private static Turret createTestTurret(
      FakeAzimuthIO azimuthIO, double minAzimuthDeg, double maxAzimuthDeg) {
    return new Turret(
        "TestTurret",
        new Azimuth("Test/Azimuth", azimuthIO),
        new Hood("Test/Hood", new FakeHoodIO()),
        new Flywheels("Test/Flywheels", new FakeFlywheelsIO()),
        minAzimuthDeg,
        maxAzimuthDeg);
  }
}
