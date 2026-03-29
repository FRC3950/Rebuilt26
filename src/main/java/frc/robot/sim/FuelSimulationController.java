package frc.robot.sim;

import static frc.robot.Constants.SimConstants.Fuel.BUMPER_HEIGHT_METERS;
import static frc.robot.Constants.SimConstants.Fuel.INTAKE_X_MAX_METERS;
import static frc.robot.Constants.SimConstants.Fuel.INTAKE_X_MIN_METERS;
import static frc.robot.Constants.SimConstants.Fuel.INTAKE_Y_MAX_METERS;
import static frc.robot.Constants.SimConstants.Fuel.INTAKE_Y_MIN_METERS;
import static frc.robot.Constants.SimConstants.Fuel.MAX_FUEL_CAPACITY;
import static frc.robot.Constants.SimConstants.Fuel.OUTTAKE_BALLS_PER_SECOND;
import static frc.robot.Constants.SimConstants.Fuel.OUTTAKE_SPEED_METERS_PER_SECOND;
import static frc.robot.Constants.SimConstants.Fuel.OUTTAKE_WAVE_MAX_BALLS;
import static frc.robot.Constants.SimConstants.Fuel.OUTTAKE_WAVE_MIN_BALLS;
import static frc.robot.Constants.SimConstants.Fuel.OUTTAKE_WAVE_SPACING_METERS;
import static frc.robot.Constants.SimConstants.Fuel.ROBOT_LENGTH_METERS;
import static frc.robot.Constants.SimConstants.Fuel.ROBOT_WIDTH_METERS;
import static frc.robot.Constants.SimConstants.Fuel.SHOOT_BALLS_PER_SECOND_PER_TURRET;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ThreadLocalRandom;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class FuelSimulationController {
  public record TurretSimSource(
      Translation2d robotToTurret,
      DoubleSupplier azimuthDegSupplier,
      DoubleSupplier hoodAngleDegSupplier,
      DoubleSupplier flywheelRpsSupplier) {}

  private final FuelSim fuelSim;
  private final FuelLaunchCalculator launchCalculator;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;
  private final DoubleSupplier intakeRollerSpeedSupplier;
  private final BooleanSupplier intakeDownSupplier;
  private final BooleanSupplier shootingSupplier;
  private final IntSupplier outtakeWaveSizeSupplier;
  private final List<TurretSimSource> turretSources;
  private final boolean spawnStartingFuelOnInitialize;

  private final double[] shotAccumulators;
  private double outtakeAccumulator = 0.0;
  private int currentFuelCapacity = 0;

  public FuelSimulationController(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier,
      DoubleSupplier intakeRollerSpeedSupplier,
      BooleanSupplier intakeDownSupplier,
      BooleanSupplier shootingSupplier,
      TurretSimSource... turretSources) {
    this(
        new FuelSim(),
        new FuelLaunchCalculator(),
        robotPoseSupplier,
        fieldSpeedsSupplier,
        intakeRollerSpeedSupplier,
        intakeDownSupplier,
        shootingSupplier,
        () ->
            ThreadLocalRandom.current().nextInt(OUTTAKE_WAVE_MIN_BALLS, OUTTAKE_WAVE_MAX_BALLS + 1),
        true,
        turretSources);
  }

  FuelSimulationController(
      FuelSim fuelSim,
      FuelLaunchCalculator launchCalculator,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier,
      DoubleSupplier intakeRollerSpeedSupplier,
      BooleanSupplier intakeDownSupplier,
      BooleanSupplier shootingSupplier,
      IntSupplier outtakeWaveSizeSupplier,
      boolean spawnStartingFuelOnInitialize,
      TurretSimSource... turretSources) {
    this.fuelSim = fuelSim;
    this.launchCalculator = launchCalculator;
    this.robotPoseSupplier = robotPoseSupplier;
    this.fieldSpeedsSupplier = fieldSpeedsSupplier;
    this.intakeRollerSpeedSupplier = intakeRollerSpeedSupplier;
    this.intakeDownSupplier = intakeDownSupplier;
    this.shootingSupplier = shootingSupplier;
    this.outtakeWaveSizeSupplier = outtakeWaveSizeSupplier;
    this.turretSources = List.of(turretSources);
    this.spawnStartingFuelOnInitialize = spawnStartingFuelOnInitialize;
    this.shotAccumulators = new double[turretSources.length];

    configureFuelSim();
  }

  private void configureFuelSim() {
    fuelSim.registerRobot(
        ROBOT_WIDTH_METERS,
        ROBOT_LENGTH_METERS,
        BUMPER_HEIGHT_METERS,
        robotPoseSupplier,
        fieldSpeedsSupplier);
    fuelSim.registerIntake(
        INTAKE_X_MIN_METERS,
        INTAKE_X_MAX_METERS,
        INTAKE_Y_MIN_METERS,
        INTAKE_Y_MAX_METERS,
        this::canIntakeFuel,
        this::storeFuel);
    fuelSim.enableAirResistance();
  }

  public void initializeSimulation() {
    fuelSim.stop();
    resetFieldFuel(spawnStartingFuelOnInitialize);
    currentFuelCapacity = 0;
    outtakeAccumulator = 0.0;
    Arrays.fill(shotAccumulators, 0.0);
    fuelSim.start();
    logState();
  }

  public void resetFieldFuelToStartingConfiguration() {
    resetFieldFuel(true);
  }

  private void resetFieldFuel(boolean spawnStartingFuel) {
    fuelSim.clearFuel();
    if (spawnStartingFuel) {
      fuelSim.spawnStartingFuel();
    }
    FuelSim.Hub.BLUE_HUB.resetScore();
    FuelSim.Hub.RED_HUB.resetScore();
    logState();
  }

  public void stepSimulation() {
    fuelSim.updateSim();

    Pose2d robotPose = robotPoseSupplier.get();
    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
    processTurretShooting(robotPose, fieldSpeeds);
    processOuttake(robotPose, fieldSpeeds);
    logState();
  }

  public void stopSimulation() {
    fuelSim.stop();
    outtakeAccumulator = 0.0;
    Arrays.fill(shotAccumulators, 0.0);
  }

  public int getCurrentFuelCapacity() {
    return currentFuelCapacity;
  }

  private boolean canIntakeFuel() {
    return intakeRollerSpeedSupplier.getAsDouble() > 0.0
        && intakeDownSupplier.getAsBoolean()
        && currentFuelCapacity < MAX_FUEL_CAPACITY;
  }

  private void storeFuel() {
    if (currentFuelCapacity < MAX_FUEL_CAPACITY) {
      currentFuelCapacity++;
    }
  }

  private void processTurretShooting(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {
    if (!shootingSupplier.getAsBoolean()) {
      Arrays.fill(shotAccumulators, 0.0);
      return;
    }

    for (int i = 0; i < turretSources.size(); i++) {
      shotAccumulators[i] += SHOOT_BALLS_PER_SECOND_PER_TURRET * Constants.loopPeriodSecs;
      while (shotAccumulators[i] >= 1.0 && currentFuelCapacity > 0) {
        TurretSimSource turretSource = turretSources.get(i);
        var launch =
            launchCalculator.calculateTurretLaunch(
                robotPose,
                fieldSpeeds,
                turretSource.robotToTurret(),
                turretSource.azimuthDegSupplier().getAsDouble(),
                turretSource.hoodAngleDegSupplier().getAsDouble(),
                turretSource.flywheelRpsSupplier().getAsDouble());
        fuelSim.spawnFuel(launch.position(), launch.velocity());
        currentFuelCapacity--;
        shotAccumulators[i] -= 1.0;
      }

      if (currentFuelCapacity == 0) {
        shotAccumulators[i] = 0.0;
      }
    }
  }

  private void processOuttake(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {
    if (!canOuttakeFuel()) {
      outtakeAccumulator = 0.0;
      return;
    }

    outtakeAccumulator += OUTTAKE_BALLS_PER_SECOND * Constants.loopPeriodSecs;
    while (outtakeAccumulator >= 1.0 && currentFuelCapacity > 0) {
      int waveBallCount =
          Math.min(
              currentFuelCapacity,
              Math.max(
                  OUTTAKE_WAVE_MIN_BALLS,
                  Math.min(OUTTAKE_WAVE_MAX_BALLS, outtakeWaveSizeSupplier.getAsInt())));
      for (int i = 0; i < waveBallCount; i++) {
        double lateralOffsetMeters = (i - (waveBallCount - 1) / 2.0) * OUTTAKE_WAVE_SPACING_METERS;
        var launch =
            launchCalculator.calculateOuttakeLaunch(
                robotPose,
                fieldSpeeds,
                FuelSim.FUEL_RADIUS,
                OUTTAKE_SPEED_METERS_PER_SECOND,
                lateralOffsetMeters);
        fuelSim.spawnFuel(launch.position(), launch.velocity());
        currentFuelCapacity--;
      }
      outtakeAccumulator -= 1.0;
    }

    if (currentFuelCapacity == 0) {
      outtakeAccumulator = 0.0;
    }
  }

  private boolean canOuttakeFuel() {
    return intakeRollerSpeedSupplier.getAsDouble() < 0.0
        && intakeDownSupplier.getAsBoolean()
        && currentFuelCapacity > 0;
  }

  private void logState() {
    Logger.recordOutput("FuelSim/StoredFuel", currentFuelCapacity);
    Logger.recordOutput("FuelSim/MaxFuelCapacity", MAX_FUEL_CAPACITY);
    Logger.recordOutput("FuelSim/ShootingActive", shootingSupplier.getAsBoolean());
    Logger.recordOutput("FuelSim/OuttakeActive", canOuttakeFuel());
    Logger.recordOutput("FuelSim/BlueHubScore", FuelSim.Hub.BLUE_HUB.getScore());
    Logger.recordOutput("FuelSim/RedHubScore", FuelSim.Hub.RED_HUB.getScore());
  }
}
