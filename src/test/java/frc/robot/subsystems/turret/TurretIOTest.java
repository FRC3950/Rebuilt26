package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
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
    Turret turret =
        new Turret(
            "TestTurret",
            new Azimuth("Test/Azimuth", new FakeAzimuthIO()),
            new Hood("Test/Hood", new FakeHoodIO()),
            new Flywheels("Test/Flywheels", new FakeFlywheelsIO()),
            -180.0,
            180.0);

    turret.runSetpoints(Rotation2d.kZero, 45.0, 60.0);

    assertEquals(
        frc.robot.Constants.SubsystemConstants.Turret.maxHoodAngle,
        turret.getRequestedHoodAngleDeg());
  }
}
