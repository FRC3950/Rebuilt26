package frc.robot.sim;

import static frc.robot.Constants.FieldConstants.hubGuy;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret1;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.ShotTable2d;
import org.junit.jupiter.api.Test;

class GeneratedShotTableSimulationTest {
  @Test
  void generatedMidRangeShotScoresInFuelSimLaunchModel() {
    ShotTable2d table = ShotTable2d.loadFromDeploy().orElseThrow();
    var sample = table.interpolateSample(4.0, 0.0);
    Pose2d robotPose =
        new Pose2d(
            hubGuy.getX() - 4.0 - robotToTurret1.getX(),
            hubGuy.getY() - robotToTurret1.getY(),
            Rotation2d.kZero);
    FuelLaunchCalculator calculator = new FuelLaunchCalculator();
    var launch =
        calculator.calculateTurretLaunch(
            robotPose,
            new ChassisSpeeds(),
            robotToTurret1,
            0.0,
            sample.hoodDeg(),
            sample.flywheelRps());

    FuelSim fuelSim = new FuelSim("/GeneratedShotTableSimulationTest");
    fuelSim.setLoggingFrequency(Double.MAX_VALUE);
    fuelSim.setSubticks(10);
    fuelSim.enableAirResistance();
    FuelSim.Hub.BLUE_HUB.resetScore();
    FuelSim.Hub.RED_HUB.resetScore();
    fuelSim.spawnFuel(launch.position(), launch.velocity());
    fuelSim.start();
    for (int i = 0; i < 180; i++) {
      fuelSim.stepSim();
    }
    fuelSim.stop();

    assertTrue(
        FuelSim.Hub.BLUE_HUB.getScore() > 0,
        () ->
            "Generated shot missed in FuelSim: sample="
                + sample
                + ", launch="
                + launch
                + ", score="
                + FuelSim.Hub.BLUE_HUB.getScore());
  }
}
