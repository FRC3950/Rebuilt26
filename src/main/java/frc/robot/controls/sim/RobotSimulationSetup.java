package frc.robot.controls.sim;

import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret1;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.DemoContainer;
import frc.robot.sim.FuelSimCommand;
import frc.robot.sim.FuelSimulationController;
import frc.robot.sim.RobotBumpSim;
import frc.robot.sim.RobotFieldCollisionSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import java.util.function.Supplier;

public final class RobotSimulationSetup {
  public record Runtime(RobotSimulationControls controls, Command command) {}

  private RobotSimulationSetup() {}

  public static Runtime create(
      Runnable enableDemoMode,
      Supplier<DemoContainer> demoContainerSupplier,
      Drive drive,
      Intake intake,
      Indexer indexer,
      Turret turret1,
      Turret turret2) {
    if (Constants.currentMode != Constants.Mode.SIM) {
      return new Runtime(null, null);
    }

    RobotSimulationControls controls =
        new RobotSimulationControls(enableDemoMode, demoContainerSupplier, drive, intake, indexer);
    FuelSimulationController fuelSimulationController =
        new FuelSimulationController(
            drive::getPose,
            drive::getFieldRelativeSpeeds,
            intake::getCommandedRollerSpeed,
            intake::isPivotCommandedDown,
            indexer::isFeedingForward,
            new FuelSimulationController.TurretSimSource(
                robotToTurret1,
                turret1::getCommandedAzimuthDeg,
                turret1::getCommandedHoodAngleDeg,
                turret1::getCommandedFlywheelRps),
            new FuelSimulationController.TurretSimSource(
                robotToTurret2,
                turret2::getCommandedAzimuthDeg,
                turret2::getCommandedHoodAngleDeg,
                turret2::getCommandedFlywheelRps));

    RobotBumpSim robotBumpSim = new RobotBumpSim(Drive.getModuleTranslations());
    drive.setSimDynamics(
        (pose, fieldRelativeSpeeds) -> {
          Pose2d collisionConstrainedPose = RobotFieldCollisionSim.constrainRobotPose(pose);
          Pose3d terrainPose =
              robotBumpSim.update(
                  collisionConstrainedPose, fieldRelativeSpeeds, RobotBumpSim.DEFAULT_SUBTICKS);
          Pose2d correctedPose =
              robotBumpSim.isOnRamp()
                  ? robotBumpSim.getSimWorldPose(collisionConstrainedPose)
                  : collisionConstrainedPose;
          return new Drive.SimDynamicsResult(correctedPose, terrainPose, robotBumpSim.isOnRamp());
        });

    SmartDashboard.putBoolean(RobotSimulationControls.UNLIMITED_FUEL_CAPACITY_DASHBOARD_KEY, false);
    Command simulationCommand =
        new FuelSimCommand(
            fuelSimulationController,
            () ->
                SmartDashboard.getBoolean(
                    RobotSimulationControls.UNLIMITED_FUEL_CAPACITY_DASHBOARD_KEY, false));
    controls.setFuelSimulationController(fuelSimulationController);
    SmartDashboard.putData(
        "Fuel Sim/Reset Field Fuel",
        Commands.runOnce(fuelSimulationController::resetFieldFuelToStartingConfiguration)
            .ignoringDisable(true));

    return new Runtime(controls, simulationCommand);
  }
}
