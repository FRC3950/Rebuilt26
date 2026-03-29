package frc.robot.sim;

import static frc.robot.Constants.SimConstants.Fuel.INTAKE_CENTER;
import static frc.robot.Constants.SimConstants.Fuel.MAX_FUEL_CAPACITY;
import static frc.robot.Constants.SimConstants.Fuel.OUTTAKE_SPEED_METERS_PER_SECOND;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret1;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret2;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;

class FuelSimulationControllerTest {
  private static final double EPSILON = 1e-9;

  @Test
  void intakeStopsAtMaxCapacityAndLeavesExtraFuelOnField() {
    TestContext context = new TestContext();
    context.controller.initializeSimulation();
    context.intakeDown.value = true;
    context.intakeRollerSpeed.value = 40.0;

    for (int i = 0; i < MAX_FUEL_CAPACITY + 1; i++) {
      spawnFuelInIntake(context, -0.12 + 0.05 * i);
    }

    context.controller.stepSimulation();

    assertEquals(MAX_FUEL_CAPACITY, context.controller.getCurrentFuelCapacity());
    assertEquals(1, context.fuelSim.fuels.size());
  }

  @Test
  void dualTurretShootingDrainsSharedCapacityAtDoubleRate() {
    TestContext context = new TestContext();
    context.controller.initializeSimulation();
    context.intakeDown.value = true;
    context.intakeRollerSpeed.value = 40.0;

    spawnFuelInIntake(context, -0.05);
    spawnFuelInIntake(context, 0.05);
    context.controller.stepSimulation();
    assertEquals(2, context.controller.getCurrentFuelCapacity());

    context.intakeRollerSpeed.value = 0.0;
    context.shooting.value = true;
    int spawnCountBeforeShooting = context.fuelSim.spawnCount;
    for (int i = 0; i < 13; i++) {
      context.controller.stepSimulation();
    }

    assertEquals(0, context.controller.getCurrentFuelCapacity());
    assertEquals(2, context.fuelSim.spawnCount - spawnCountBeforeShooting);
  }

  @Test
  void outtakeSpawnsFuelFromIntakeAndDrainsStoredCapacity() {
    TestContext context = new TestContext();
    context.controller.initializeSimulation();
    context.intakeDown.value = true;
    context.intakeRollerSpeed.value = 40.0;

    spawnFuelInIntake(context, 0.0);
    context.controller.stepSimulation();
    assertEquals(1, context.controller.getCurrentFuelCapacity());

    context.intakeRollerSpeed.value = -45.0;
    for (int i = 0; i < 13; i++) {
      context.controller.stepSimulation();
    }

    assertEquals(0, context.controller.getCurrentFuelCapacity());
    assertEquals(INTAKE_CENTER.getX(), context.fuelSim.lastSpawnPosition.getX(), EPSILON);
    assertEquals(INTAKE_CENTER.getY(), context.fuelSim.lastSpawnPosition.getY(), EPSILON);
    assertEquals(FuelSim.FUEL_RADIUS, context.fuelSim.lastSpawnPosition.getZ(), EPSILON);
    assertTrue(context.fuelSim.lastSpawnVelocity.getX() >= OUTTAKE_SPEED_METERS_PER_SECOND);
  }

  @Test
  void commandInitializesStepsAndStopsController() {
    TestContext context = new TestContext();
    FuelSimCommand command = new FuelSimCommand(context.controller);

    command.initialize();
    int stopCallsAfterInitialize = context.fuelSim.stopCalls;
    command.execute();
    command.end(false);

    assertEquals(1, context.fuelSim.startCalls);
    assertEquals(1, context.fuelSim.updateCalls);
    assertEquals(stopCallsAfterInitialize + 1, context.fuelSim.stopCalls);
  }

  private static void spawnFuelInIntake(TestContext context, double yOffsetMeters) {
    context.fuelSim.spawnFuel(
        new Translation3d(
            INTAKE_CENTER.getX(), INTAKE_CENTER.getY() + yOffsetMeters, FuelSim.FUEL_RADIUS),
        new Translation3d());
  }

  private static class TestContext {
    final MutablePoseSupplier pose = new MutablePoseSupplier();
    final MutableChassisSpeedsSupplier speeds = new MutableChassisSpeedsSupplier();
    final MutableDoubleSupplier intakeRollerSpeed = new MutableDoubleSupplier();
    final MutableBooleanSupplier intakeDown = new MutableBooleanSupplier();
    final MutableBooleanSupplier shooting = new MutableBooleanSupplier();
    final TestFuelSim fuelSim = new TestFuelSim();
    final FuelSimulationController controller =
        new FuelSimulationController(
            fuelSim,
            new FuelLaunchCalculator(),
            pose,
            speeds,
            intakeRollerSpeed,
            intakeDown,
            shooting,
            false,
            new FuelSimulationController.TurretSimSource(
                robotToTurret1, () -> 0.0, () -> 20.0, () -> 30.0),
            new FuelSimulationController.TurretSimSource(
                robotToTurret2, () -> 0.0, () -> 20.0, () -> 30.0));

    TestContext() {
      pose.value = new Pose2d(0.0, 0.0, Rotation2d.kZero);
      speeds.value = new ChassisSpeeds();
    }
  }

  private static class MutablePoseSupplier implements Supplier<Pose2d> {
    Pose2d value = Pose2d.kZero;

    @Override
    public Pose2d get() {
      return value;
    }
  }

  private static class MutableChassisSpeedsSupplier implements Supplier<ChassisSpeeds> {
    ChassisSpeeds value = new ChassisSpeeds();

    @Override
    public ChassisSpeeds get() {
      return value;
    }
  }

  private static class MutableDoubleSupplier implements DoubleSupplier {
    double value = 0.0;

    @Override
    public double getAsDouble() {
      return value;
    }
  }

  private static class MutableBooleanSupplier implements BooleanSupplier {
    boolean value = false;

    @Override
    public boolean getAsBoolean() {
      return value;
    }
  }

  private static class TestFuelSim extends FuelSim {
    int startCalls = 0;
    int stopCalls = 0;
    int updateCalls = 0;
    int spawnCount = 0;
    Translation3d lastSpawnPosition = new Translation3d();
    Translation3d lastSpawnVelocity = new Translation3d();

    TestFuelSim() {
      super("/FuelSimulationTest");
      setLoggingFrequency(Double.MAX_VALUE);
    }

    @Override
    public void start() {
      startCalls++;
      super.start();
    }

    @Override
    public void stop() {
      stopCalls++;
      super.stop();
    }

    @Override
    public void updateSim() {
      updateCalls++;
      super.updateSim();
    }

    @Override
    public void spawnFuel(Translation3d pos, Translation3d vel) {
      spawnCount++;
      lastSpawnPosition = pos;
      lastSpawnVelocity = vel;
      super.spawnFuel(pos, vel);
    }
  }
}
