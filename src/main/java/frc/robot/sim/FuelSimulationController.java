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
import edu.wpi.first.math.geometry.Translation3d;
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
  private int spawnedFuelCount = 0;
  private boolean unlimitedFuelCapacity = false;

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
    fuelSim.setDeleteScoredFuel(unlimitedFuelCapacity);
    resetFieldFuel(spawnStartingFuelOnInitialize);
    currentFuelCapacity = 0;
    spawnedFuelCount = 0;
    outtakeAccumulator = 0.0;
    Arrays.fill(shotAccumulators, 0.0);
    fuelSim.start();
    logState();
  }

  public void resetFieldFuelToStartingConfiguration() {
    resetFieldFuel(true);
  }

  public void clearFieldFuelForSim() {
    resetFieldFuel(false);
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

  public void setStoredFuelForSim(int storedFuel) {
    currentFuelCapacity = Math.max(0, Math.min(MAX_FUEL_CAPACITY, storedFuel));
    logState();
  }

  public void setUnlimitedFuelCapacityForSim(boolean unlimitedFuelCapacity) {
    if (unlimitedFuelCapacity && !this.unlimitedFuelCapacity) {
      fuelSim.clearFuel();
    }
    this.unlimitedFuelCapacity = unlimitedFuelCapacity;
    fuelSim.setDeleteScoredFuel(unlimitedFuelCapacity);
    logState();
  }

  public boolean isUnlimitedFuelCapacityForSim() {
    return unlimitedFuelCapacity;
  }

  public int getSpawnedFuelCountForSim() {
    return spawnedFuelCount;
  }

  public int getFieldFuelCountForSim() {
    return fuelSim.getFuelCount();
  }

  public List<Translation3d> getFieldFuelPositionsForSim() {
    return fuelSim.getFuelPositionsSnapshot();
  }

  private boolean canIntakeFuel() {
    return intakeRollerSpeedSupplier.getAsDouble() > 0.0
        && intakeDownSupplier.getAsBoolean()
        && (unlimitedFuelCapacity || currentFuelCapacity < MAX_FUEL_CAPACITY);
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
      while (shotAccumulators[i] >= 1.0 && hasFuelAvailable()) {
        TurretSimSource turretSource = turretSources.get(i);
        var launch =
            launchCalculator.calculateTurretLaunch(
                robotPose,
                fieldSpeeds,
                turretSource.robotToTurret(),
                turretSource.azimuthDegSupplier().getAsDouble(),
                turretSource.hoodAngleDegSupplier().getAsDouble(),
                turretSource.flywheelRpsSupplier().getAsDouble());
        spawnSimFuel(launch.position(), launch.velocity());
        consumeFuelIfLimited();
        shotAccumulators[i] -= 1.0;
      }

      if (!hasFuelAvailable()) {
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
    while (outtakeAccumulator >= 1.0 && hasFuelAvailable()) {
      int waveBallCount = getOuttakeWaveBallCount();
      for (int i = 0; i < waveBallCount; i++) {
        double lateralOffsetMeters = (i - (waveBallCount - 1) / 2.0) * OUTTAKE_WAVE_SPACING_METERS;
        var launch =
            launchCalculator.calculateOuttakeLaunch(
                robotPose,
                fieldSpeeds,
                FuelSim.FUEL_RADIUS,
                OUTTAKE_SPEED_METERS_PER_SECOND,
                lateralOffsetMeters);
        spawnSimFuel(launch.position(), launch.velocity());
        consumeFuelIfLimited();
      }
      outtakeAccumulator -= 1.0;
    }

    if (!hasFuelAvailable()) {
      outtakeAccumulator = 0.0;
    }
  }

  private boolean canOuttakeFuel() {
    return intakeRollerSpeedSupplier.getAsDouble() < 0.0
        && intakeDownSupplier.getAsBoolean()
        && hasFuelAvailable();
  }

  private boolean hasFuelAvailable() {
    return unlimitedFuelCapacity || currentFuelCapacity > 0;
  }

  private void consumeFuelIfLimited() {
    if (!unlimitedFuelCapacity) {
      currentFuelCapacity--;
    }
  }

  private void spawnSimFuel(Translation3d position, Translation3d velocity) {
    fuelSim.spawnFuel(position, velocity);
    spawnedFuelCount++;
  }

  private int getOuttakeWaveBallCount() {
    int requestedWaveBallCount =
        Math.max(
            OUTTAKE_WAVE_MIN_BALLS,
            Math.min(OUTTAKE_WAVE_MAX_BALLS, outtakeWaveSizeSupplier.getAsInt()));
    return unlimitedFuelCapacity
        ? requestedWaveBallCount
        : Math.min(currentFuelCapacity, requestedWaveBallCount);
  }

  private void logState() {
    Logger.recordOutput("FuelSim/StoredFuel", currentFuelCapacity);
    Logger.recordOutput("FuelSim/SpawnedFuelCount", spawnedFuelCount);
    Logger.recordOutput("FuelSim/FieldFuelCount", fuelSim.getFuelCount());
    Logger.recordOutput("FuelSim/MaxFuelCapacity", MAX_FUEL_CAPACITY);
    Logger.recordOutput("FuelSim/UnlimitedFuelCapacity", unlimitedFuelCapacity);
    Logger.recordOutput("FuelSim/ShootingActive", shootingSupplier.getAsBoolean());
    Logger.recordOutput("FuelSim/OuttakeActive", canOuttakeFuel());
    Logger.recordOutput("FuelSim/BlueHubScore", FuelSim.Hub.BLUE_HUB.getScore());
    Logger.recordOutput("FuelSim/RedHubScore", FuelSim.Hub.RED_HUB.getScore());
  }
}
