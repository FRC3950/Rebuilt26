package frc.robot.controls.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SubsystemConstants.Turret;
import frc.robot.DemoContainer;
import frc.robot.sim.FieldTerrain;
import frc.robot.sim.FuelSim;
import frc.robot.sim.FuelSimulationController;
import frc.robot.sim.RobotFieldCollisionSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RobotSimulationControls {
  public static final String UNLIMITED_FUEL_CAPACITY_DASHBOARD_KEY =
      "Fuel Sim/Unlimited Fuel Capacity";

  private final Runnable enableDemoMode;
  private final Supplier<DemoContainer> demoContainerSupplier;
  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;
  private FuelSimulationController fuelSimulationController;
  private int shotMapSweepSamplesPassed = 0;
  private int shotMapSweepSamplesTotal = 0;
  private double ferryShotClosestGroundDistanceMeters = Double.POSITIVE_INFINITY;
  private boolean ferryShotAnyGroundContactOnBump = false;
  private int trenchCollisionSamplesPassed = 0;
  private int trenchCollisionSamplesTotal = 0;

  public RobotSimulationControls(
      Runnable enableDemoMode,
      Supplier<DemoContainer> demoContainerSupplier,
      Drive drive,
      Intake intake,
      Indexer indexer) {
    this.enableDemoMode = enableDemoMode;
    this.demoContainerSupplier = demoContainerSupplier;
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
  }

  public void setFuelSimulationController(FuelSimulationController fuelSimulationController) {
    this.fuelSimulationController = fuelSimulationController;
  }

  public void enableDemoMode() {
    enableDemoMode.run();
  }

  public void setHandheldTagTargetingEnabled(boolean enabled) {
    getOrEnableDemoContainer().setHandheldTagTargetingEnabled(enabled);
  }

  public void setTargetTagId(int tagId) {
    getOrEnableDemoContainer().setTargetTagId(tagId);
  }

  public void setPose(double xMeters, double yMeters, double headingDeg) {
    getOrEnableDemoContainer().setSimPose(xMeters, yMeters, headingDeg);
  }

  public void setHandheldTagObservation(int tagId, double txncDeg, double distanceToRobotMeters) {
    getOrEnableDemoContainer().setSimHandheldTagObservation(tagId, txncDeg, distanceToRobotMeters);
  }

  public void recordHandheldTagState() {
    DemoContainer demoContainer = demoContainerSupplier.get();
    if (demoContainer == null) {
      Logger.recordOutput("GymSim/DemoHandheld/TargetValid", false);
      Logger.recordOutput("GymSim/DemoHandheld/TargetValidNumeric", 0.0);
      Logger.recordOutput("GymSim/DemoHandheld/PoseFusionAllowed", true);
      Logger.recordOutput("GymSim/DemoHandheld/PoseFusionBlockedNumeric", 0.0);
      return;
    }
    demoContainer.recordHandheldTagStateForSim();
  }

  public Command generatedShotValidationCommand(
      double robotXMeters,
      double robotYMeters,
      double headingDeg,
      int storedFuel,
      double durationSeconds) {
    if (Constants.currentMode != Constants.Mode.SIM || fuelSimulationController == null) {
      return Commands.none();
    }

    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  setPose(robotXMeters, robotYMeters, headingDeg);
                  FuelSim.Hub.BLUE_HUB.resetScore();
                  FuelSim.Hub.RED_HUB.resetScore();
                  fuelSimulationController.setStoredFuelForSim(storedFuel);
                  recordGeneratedShotValidationState();
                }),
            Commands.runEnd(
                    () -> {
                      indexer.requestForwardFeed();
                      intake.startIntake();
                    },
                    () -> {
                      indexer.stopForwardFeed();
                      intake.stopIntake();
                    },
                    indexer,
                    intake)
                .withTimeout(Math.max(0.1, durationSeconds)),
            Commands.runOnce(this::recordGeneratedShotValidationState))
        .withName("Generated Shot Map Validation");
  }

  public Command generatedShotMapSweepValidationCommand() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  shotMapSweepSamplesPassed = 0;
                  shotMapSweepSamplesTotal = 0;
                  recordGeneratedShotValidationState();
                }),
            generatedShotValidationCommandForDistance(2.0),
            Commands.runOnce(() -> recordShotMapSweepSample(2.0)),
            generatedShotValidationCommandForDistance(4.0),
            Commands.runOnce(() -> recordShotMapSweepSample(4.0)),
            generatedShotValidationCommandForDistance(6.0),
            Commands.runOnce(() -> recordShotMapSweepSample(6.0)),
            Commands.runOnce(this::recordGeneratedShotValidationState))
        .withName("Generated Shot Map Sweep Validation");
  }

  public Command ferryShotValidationCommand(double durationSeconds) {
    if (Constants.currentMode != Constants.Mode.SIM || fuelSimulationController == null) {
      return Commands.none();
    }

    Translation2d target = FieldConstants.getLeftFerryTarget();
    double robotXMeters = target.getX() + 3.0 - Turret.robotToTurret1.getX();
    double robotYMeters = target.getY() - Turret.robotToTurret1.getY();

    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  setPose(robotXMeters, robotYMeters, 0.0);
                  fuelSimulationController.clearFieldFuelForSim();
                  fuelSimulationController.setStoredFuelForSim(2);
                  ferryShotClosestGroundDistanceMeters = Double.POSITIVE_INFINITY;
                  ferryShotAnyGroundContactOnBump = false;
                  recordFerryShotValidationState();
                }),
            Commands.runEnd(
                    () -> {
                      indexer.requestForwardFeed();
                      intake.startIntake();
                      updateFerryShotClosestGroundDistance(target);
                    },
                    () -> {
                      indexer.stopForwardFeed();
                      intake.stopIntake();
                    },
                    indexer,
                    intake)
                .withTimeout(Math.max(0.1, durationSeconds)),
            Commands.runOnce(this::recordFerryShotValidationState))
        .withName("Ferry Shot Map Validation");
  }

  public Command ferryShotMovingAwayValidationCommand(
      double durationSeconds, double distanceMeters, double awayVelocityMetersPerSec) {
    if (Constants.currentMode != Constants.Mode.SIM || fuelSimulationController == null) {
      return Commands.none();
    }

    Translation2d target = FieldConstants.getLeftFerryTarget();
    double robotXMeters = target.getX() + distanceMeters - Turret.robotToTurret1.getX();
    double robotYMeters = target.getY() - Turret.robotToTurret1.getY();

    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  setPose(robotXMeters, robotYMeters, 0.0);
                  fuelSimulationController.clearFieldFuelForSim();
                  fuelSimulationController.setStoredFuelForSim(2);
                  ferryShotClosestGroundDistanceMeters = Double.POSITIVE_INFINITY;
                  ferryShotAnyGroundContactOnBump = false;
                  recordFerryShotValidationState();
                }),
            Commands.runEnd(
                    () -> {
                      drive.runVelocity(new ChassisSpeeds(awayVelocityMetersPerSec, 0.0, 0.0));
                      indexer.requestForwardFeed();
                      intake.startIntake();
                      updateFerryShotClosestGroundDistance(target);
                    },
                    () -> {
                      drive.stop();
                      indexer.stopForwardFeed();
                      intake.stopIntake();
                    },
                    drive,
                    indexer,
                    intake)
                .withTimeout(Math.max(0.1, durationSeconds)),
            Commands.runOnce(this::recordFerryShotValidationState))
        .withName("Moving Away Ferry Shot Map Validation");
  }

  public void recordGeneratedShotValidationState() {
    Logger.recordOutput("GymSim/ShotMap/StoredFuel", getStoredFuelForLog());
    Logger.recordOutput("GymSim/ShotMap/BlueHubScore", FuelSim.Hub.BLUE_HUB.getScore());
    Logger.recordOutput("GymSim/ShotMap/RedHubScore", FuelSim.Hub.RED_HUB.getScore());
    Logger.recordOutput("GymSim/ShotMap/SweepSamplesPassed", shotMapSweepSamplesPassed);
    Logger.recordOutput("GymSim/ShotMap/SweepSamplesTotal", shotMapSweepSamplesTotal);
    Logger.recordOutput(
        "GymSim/ShotMap/SweepPassedNumeric",
        shotMapSweepSamplesTotal >= 3 && shotMapSweepSamplesPassed == shotMapSweepSamplesTotal
            ? 1.0
            : 0.0);
  }

  public void recordFerryShotValidationState() {
    updateFerryShotClosestGroundDistance(FieldConstants.getLeftFerryTarget());
    Logger.recordOutput(
        "GymSim/FerryShotMap/ClosestGroundDistanceMeters", ferryShotClosestGroundDistanceMeters);
    Logger.recordOutput(
        "GymSim/FerryShotMap/PassedNumeric",
        ferryShotClosestGroundDistanceMeters <= 0.75 && !ferryShotAnyGroundContactOnBump
            ? 1.0
            : 0.0);
    Logger.recordOutput(
        "GymSim/FerryShotMap/AnyGroundContactOnBumpNumeric",
        ferryShotAnyGroundContactOnBump ? 1.0 : 0.0);
    Logger.recordOutput("GymSim/FerryShotMap/StoredFuel", getStoredFuelForLog());
  }

  public void setUnlimitedFuelCapacityEnabled(boolean enabled) {
    SmartDashboard.putBoolean(UNLIMITED_FUEL_CAPACITY_DASHBOARD_KEY, enabled);
    if (fuelSimulationController != null) {
      fuelSimulationController.setUnlimitedFuelCapacityForSim(enabled);
    }
  }

  public Command unlimitedFuelCapacityValidationCommand(double durationSeconds) {
    if (Constants.currentMode != Constants.Mode.SIM || fuelSimulationController == null) {
      return Commands.none();
    }

    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  fuelSimulationController.setStoredFuelForSim(0);
                  setUnlimitedFuelCapacityEnabled(true);
                  recordFuelSimulationState();
                }),
            Commands.runEnd(
                    () -> {
                      indexer.requestForwardFeed();
                      intake.startIntake();
                    },
                    () -> {
                      indexer.stopForwardFeed();
                      intake.stopIntake();
                    },
                    indexer,
                    intake)
                .withTimeout(Math.max(0.1, durationSeconds)),
            Commands.runOnce(this::recordFuelSimulationState))
        .withName("Unlimited Fuel Capacity Validation");
  }

  public void recordFuelSimulationState() {
    Logger.recordOutput("GymSim/FuelSim/StoredFuel", getStoredFuelForLog());
    Logger.recordOutput(
        "GymSim/FuelSim/UnlimitedFuelCapacity",
        fuelSimulationController != null && fuelSimulationController.isUnlimitedFuelCapacityForSim()
            ? 1.0
            : 0.0);
    Logger.recordOutput(
        "GymSim/FuelSim/SpawnedFuelCount",
        fuelSimulationController != null
            ? fuelSimulationController.getSpawnedFuelCountForSim()
            : 0);
    Logger.recordOutput(
        "GymSim/FuelSim/FieldFuelCount",
        fuelSimulationController != null ? fuelSimulationController.getFieldFuelCountForSim() : 0);
  }

  public void recordRobotTerrainState() {
    var robotPose = drive.getPose();
    Pose3d terrainPose = drive.getLastSimTerrainPose();
    Logger.recordOutput("GymSim/RobotTerrain/OdometryX", robotPose.getX());
    Logger.recordOutput("GymSim/RobotTerrain/OdometryY", robotPose.getY());
    Logger.recordOutput("GymSim/RobotTerrain/HeightMeters", terrainPose.getZ());
    Logger.recordOutput(
        "GymSim/RobotTerrain/RollDeg", Units.radiansToDegrees(terrainPose.getRotation().getX()));
    Logger.recordOutput(
        "GymSim/RobotTerrain/PitchDeg", Units.radiansToDegrees(terrainPose.getRotation().getY()));
  }

  public Command trenchSideCollisionValidationCommand() {
    if (Constants.currentMode != Constants.Mode.SIM) {
      return Commands.none();
    }

    Command[] steps = new Command[RobotFieldCollisionSim.getTrenchSideObstacleCountForSim() + 2];
    steps[0] =
        Commands.runOnce(
            () -> {
              trenchCollisionSamplesPassed = 0;
              trenchCollisionSamplesTotal = 0;
              recordTrenchCollisionSummary();
            });
    for (int i = 0; i < RobotFieldCollisionSim.getTrenchSideObstacleCountForSim(); i++) {
      final int sideIndex = i;
      steps[i + 1] =
          Commands.sequence(
              Commands.runOnce(() -> setPoseForTrenchSideApproach(sideIndex)),
              Commands.waitSeconds(0.04),
              Commands.runEnd(
                      () ->
                          drive.runVelocity(
                              new ChassisSpeeds(
                                  0.0,
                                  RobotFieldCollisionSim.getTrenchSideDriveYDirectionForSim(
                                          sideIndex)
                                      * 1.0,
                                  0.0)),
                      drive::stop,
                      drive)
                  .withTimeout(0.35),
              Commands.runOnce(() -> recordTrenchCollisionSample(sideIndex)));
    }
    steps[steps.length - 1] = Commands.runOnce(this::recordTrenchCollisionSummary);
    return Commands.sequence(steps).withName("Trench Side Collision Validation");
  }

  public void recordTrenchCollisionSummary() {
    Logger.recordOutput("GymSim/TrenchCollision/SamplesPassed", trenchCollisionSamplesPassed);
    Logger.recordOutput("GymSim/TrenchCollision/SamplesTotal", trenchCollisionSamplesTotal);
    Logger.recordOutput(
        "GymSim/TrenchCollision/AllPassedNumeric",
        trenchCollisionSamplesTotal == RobotFieldCollisionSim.getTrenchSideObstacleCountForSim()
                && trenchCollisionSamplesPassed == trenchCollisionSamplesTotal
            ? 1.0
            : 0.0);
  }

  private DemoContainer getOrEnableDemoContainer() {
    DemoContainer demoContainer = demoContainerSupplier.get();
    if (demoContainer == null) {
      enableDemoMode();
      demoContainer = demoContainerSupplier.get();
    }
    return demoContainer;
  }

  private int getStoredFuelForLog() {
    return fuelSimulationController != null ? fuelSimulationController.getCurrentFuelCapacity() : 0;
  }

  private void setPoseForTrenchSideApproach(int sideIndex) {
    Pose2d pose = RobotFieldCollisionSim.getTrenchSideApproachPoseForSim(sideIndex);
    setPose(pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  private void recordTrenchCollisionSample(int sideIndex) {
    Pose2d initialPose = RobotFieldCollisionSim.getTrenchSideApproachPoseForSim(sideIndex);
    Pose2d correctedPose = drive.getPose();
    boolean moved = correctedPose.getTranslation().getDistance(initialPose.getTranslation()) > 0.05;
    boolean clear =
        RobotFieldCollisionSim.robotFootprintClearsTrenchSideForSim(sideIndex, correctedPose);
    boolean stayedOnApproachSide =
        RobotFieldCollisionSim.robotRemainsOnTrenchSideApproachSideForSim(sideIndex, correctedPose);
    boolean passed = clear && stayedOnApproachSide;

    trenchCollisionSamplesTotal++;
    if (passed) {
      trenchCollisionSamplesPassed++;
    }

    String keyPrefix = "GymSim/TrenchCollision/Side" + sideIndex;
    Logger.recordOutput(keyPrefix + "/MovedNumeric", moved ? 1.0 : 0.0);
    Logger.recordOutput(keyPrefix + "/ClearNumeric", clear ? 1.0 : 0.0);
    Logger.recordOutput(
        keyPrefix + "/StayedOnApproachSideNumeric", stayedOnApproachSide ? 1.0 : 0.0);
    Logger.recordOutput(keyPrefix + "/PassedNumeric", passed ? 1.0 : 0.0);
    Logger.recordOutput(keyPrefix + "/OdometryX", correctedPose.getX());
    Logger.recordOutput(keyPrefix + "/OdometryY", correctedPose.getY());
    recordTrenchCollisionSummary();
  }

  private Command generatedShotValidationCommandForDistance(double distanceMeters) {
    double robotXMeters =
        FieldConstants.hubGuy.getX() - distanceMeters - Turret.robotToTurret1.getX();
    double robotYMeters = FieldConstants.hubGuy.getY() - Turret.robotToTurret1.getY();
    return generatedShotValidationCommand(robotXMeters, robotYMeters, 0.0, 4, 2.5);
  }

  private void updateFerryShotClosestGroundDistance(Translation2d target) {
    if (fuelSimulationController == null) {
      return;
    }
    for (var fuelPosition : fuelSimulationController.getFieldFuelPositionsForSim()) {
      Translation2d fuelTranslation = fuelPosition.toTranslation2d();
      double terrainHeightMeters = FieldTerrain.getSurfaceHeightMeters(fuelTranslation);
      if (fuelPosition.getZ() <= terrainHeightMeters + FuelSim.FUEL_RADIUS + 0.08) {
        if (FieldTerrain.isBumpFootprint(fuelTranslation)) {
          ferryShotAnyGroundContactOnBump = true;
        }
        double distance = fuelTranslation.getDistance(target);
        ferryShotClosestGroundDistanceMeters =
            Math.min(ferryShotClosestGroundDistanceMeters, distance);
      }
    }
  }

  private void recordShotMapSweepSample(double distanceMeters) {
    shotMapSweepSamplesTotal++;
    int score = FuelSim.Hub.BLUE_HUB.getScore();
    if (score > 0) {
      shotMapSweepSamplesPassed++;
    }
    Logger.recordOutput("GymSim/ShotMap/SweepLastDistanceMeters", distanceMeters);
    Logger.recordOutput("GymSim/ShotMap/SweepLastBlueHubScore", score);
    recordGeneratedShotValidationState();
  }
}
