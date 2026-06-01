package frc.robot;

import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret1;
import static frc.robot.Constants.SubsystemConstants.Turret.robotToTurret2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.controls.CompBindings;
import frc.robot.controls.CrazyBindings;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretTargeting;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.HandheldTagObservation;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class DemoContainer {
  public enum DemoBindingMode {
    COMPETITION,
    CRAZY
  }

  private static final String MAX_SPEED_KEY = "Demo Mode/Max Speed MPS";
  private static final String REDUCED_SPEED_KEY = "Demo Mode/Reduced Speed While Shooting MPS";
  private static final String HANDHELD_TAG_ENABLED_KEY = "Demo Mode/Handheld Tag Targeting Enabled";
  private static final String TARGET_TAG_ID_KEY = "Demo Mode/Target Tag ID";
  private static final double DEFAULT_MAX_SPEED_MPS = 2.0;
  private static final double TARGET_HOLD_SECS = 0.25;
  private static final double TARGET_CACHE_SECS = Constants.loopPeriodSecs * 0.5;

  private final CommandXboxController driver;
  private final Drive drive;
  private final Vision vision;
  private final Turret turret1;
  private final Turret turret2;
  private final EventLoop competitionButtonLoop = new EventLoop();
  private final EventLoop crazyButtonLoop = new EventLoop();
  private final LoggedDashboardChooser<DemoBindingMode> bindingModeChooser;

  private DemoBindingMode appliedBindingMode = DemoBindingMode.CRAZY;
  private DemoBindingMode lastPublishedSelectedBindingMode = null;
  private Translation2d lastHandheldTarget = null;
  private double lastHandheldTargetTimestampSecs = Double.NEGATIVE_INFINITY;
  private Translation2d cachedHandheldTarget = null;
  private double cachedHandheldTargetTimestampSecs = Double.NEGATIVE_INFINITY;

  public DemoContainer(
      CommandXboxController driver,
      CommandXboxController operator,
      Drive drive,
      Vision vision,
      Intake intake,
      Indexer indexer,
      Turret turret1,
      Turret turret2) {
    this.driver = driver;
    this.drive = drive;
    this.vision = vision;
    this.turret1 = turret1;
    this.turret2 = turret2;

    SmartDashboard.putNumber(MAX_SPEED_KEY, DEFAULT_MAX_SPEED_MPS);
    SmartDashboard.putNumber(REDUCED_SPEED_KEY, Constants.SubsystemConstants.Drive.reducedSpeed);
    SmartDashboard.putBoolean(HANDHELD_TAG_ENABLED_KEY, false);
    SmartDashboard.putNumber(TARGET_TAG_ID_KEY, Vision.ANY_HANDHELD_TAG_ID);

    bindingModeChooser = new LoggedDashboardChooser<>("Demo Mode/Bindings");
    bindingModeChooser.addDefaultOption("Crazy", DemoBindingMode.CRAZY);
    bindingModeChooser.addOption("Competition", DemoBindingMode.COMPETITION);

    CompBindings.configure(
        competitionButtonLoop,
        driver,
        operator,
        drive,
        intake,
        indexer,
        turret1,
        turret2,
        this::getMaxDriveSpeedMetersPerSec);
    CrazyBindings.configure(
        crazyButtonLoop,
        driver,
        drive,
        intake,
        indexer,
        turret1,
        turret2,
        this::getMaxDriveSpeedMetersPerSec);
  }

  public void checkMode(boolean isDisabled) {
    DemoBindingMode selectedBindingMode = getSelectedBindingMode();
    publishSelectedBindingMode(selectedBindingMode);

    if (!isDisabled || selectedBindingMode == appliedBindingMode) {
      return;
    }

    applySelectedBindingMode(selectedBindingMode);
  }

  public void applyCurrentBindingMode() {
    applySelectedBindingMode(getSelectedBindingMode());
  }

  public void applyDefaults() {
    turret1.setDefaultCommand(
        new TurretTargeting(turret1, drive, robotToTurret1, this::getHandheldTagTargetTranslation));
    turret2.setDefaultCommand(
        new TurretTargeting(turret2, drive, robotToTurret2, this::getHandheldTagTargetTranslation));
    drive.setDefaultCommand(
        DriveCommands.joystickDriveWithForwardDirection(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            this::getHandheldTagForwardDirection,
            drive::getMaxLinearSpeedMetersPerSec));
  }

  public double getMaxDriveSpeedMetersPerSec() {
    return sanitizeDashboardSpeed(
        SmartDashboard.getNumber(MAX_SPEED_KEY, DEFAULT_MAX_SPEED_MPS),
        DEFAULT_MAX_SPEED_MPS,
        drive.getPhysicalMaxLinearSpeedMetersPerSec());
  }

  public double getReducedSpeedMetersPerSec() {
    return sanitizeDashboardSpeed(
        SmartDashboard.getNumber(
            REDUCED_SPEED_KEY, Constants.SubsystemConstants.Drive.reducedSpeed),
        Constants.SubsystemConstants.Drive.reducedSpeed,
        getMaxDriveSpeedMetersPerSec());
  }

  public boolean isHandheldTagTargetingEnabled() {
    boolean enabled = SmartDashboard.getBoolean(HANDHELD_TAG_ENABLED_KEY, false);
    Logger.recordOutput("Controls/DemoHandheldTagTargetingEnabled", enabled);
    return enabled;
  }

  public boolean isVisionPoseFusionAllowed() {
    return !isHandheldTagTargetingEnabled();
  }

  public int getTargetTagId() {
    int targetTagId = sanitizeTargetTagId(SmartDashboard.getNumber(TARGET_TAG_ID_KEY, -1.0));
    Logger.recordOutput("Controls/DemoTargetTagId", targetTagId);
    return targetTagId;
  }

  public Translation2d getHandheldTagTargetTranslation() {
    double nowSecs = Timer.getFPGATimestamp();
    if (nowSecs - cachedHandheldTargetTimestampSecs <= TARGET_CACHE_SECS) {
      return cachedHandheldTarget;
    }

    if (!isHandheldTagTargetingEnabled()) {
      clearLoggedTarget(false);
      cachedHandheldTarget = null;
      cachedHandheldTargetTimestampSecs = nowSecs;
      return null;
    }

    Optional<HandheldTagObservation> observation =
        vision.getBestHandheldTagObservation(getTargetTagId());
    if (observation.isPresent()) {
      lastHandheldTarget = calculateTargetTranslation(drive.getPose(), observation.get());
      lastHandheldTargetTimestampSecs = nowSecs;
      cachedHandheldTarget = lastHandheldTarget;
      cachedHandheldTargetTimestampSecs = nowSecs;
      logTarget(true, lastHandheldTarget);
      return lastHandheldTarget;
    }

    if (lastHandheldTarget != null
        && nowSecs - lastHandheldTargetTimestampSecs <= TARGET_HOLD_SECS) {
      cachedHandheldTarget = lastHandheldTarget;
      cachedHandheldTargetTimestampSecs = nowSecs;
      logTarget(true, lastHandheldTarget);
      return lastHandheldTarget;
    }

    clearLoggedTarget(true);
    cachedHandheldTarget = null;
    cachedHandheldTargetTimestampSecs = nowSecs;
    return null;
  }

  public Rotation2d getHandheldTagForwardDirection() {
    Translation2d target = getHandheldTagTargetTranslation();
    if (target == null) {
      return null;
    }
    return calculateTagForwardDirection(drive.getPose(), target);
  }

  public void setHandheldTagTargetingEnabled(boolean enabled) {
    SmartDashboard.putBoolean(HANDHELD_TAG_ENABLED_KEY, enabled);
  }

  public void setTargetTagId(int tagId) {
    SmartDashboard.putNumber(TARGET_TAG_ID_KEY, tagId);
  }

  public void setSimPose(double xMeters, double yMeters, double headingDeg) {
    drive.setPose(new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(headingDeg)));
  }

  public void setSimHandheldTagObservation(
      int tagId, double txncDeg, double distanceToRobotMeters) {
    vision.setSimHandheldTagObservation(tagId, txncDeg, distanceToRobotMeters);
  }

  public void recordHandheldTagStateForSim() {
    var target = getHandheldTagTargetTranslation();
    var forwardDirection = getHandheldTagForwardDirection();
    boolean poseFusionAllowed = vision.isPoseFusionAllowed();
    Logger.recordOutput("GymSim/DemoHandheld/TargetValid", target != null);
    Logger.recordOutput("GymSim/DemoHandheld/TargetValidNumeric", target != null ? 1.0 : 0.0);
    Logger.recordOutput("GymSim/DemoHandheld/PoseFusionAllowed", poseFusionAllowed);
    Logger.recordOutput(
        "GymSim/DemoHandheld/PoseFusionBlockedNumeric", poseFusionAllowed ? 0.0 : 1.0);
    Logger.recordOutput("GymSim/DemoHandheld/LeftAzimuthDeg", turret1.getCommandedAzimuthDeg());
    Logger.recordOutput("GymSim/DemoHandheld/RightAzimuthDeg", turret2.getCommandedAzimuthDeg());
    Logger.recordOutput(
        "GymSim/DemoHandheld/ForwardDirectionDeg",
        forwardDirection != null ? forwardDirection.getDegrees() : 0.0);
    Logger.recordOutput("GymSim/DemoHandheld/TargetX", target != null ? target.getX() : 0.0);
    Logger.recordOutput("GymSim/DemoHandheld/TargetY", target != null ? target.getY() : 0.0);
  }

  public static double sanitizeDashboardSpeed(
      double dashboardSpeedMetersPerSec,
      double defaultSpeedMetersPerSec,
      double maxSpeedMetersPerSec) {
    if (!Double.isFinite(dashboardSpeedMetersPerSec) || dashboardSpeedMetersPerSec <= 0.0) {
      return Math.min(defaultSpeedMetersPerSec, maxSpeedMetersPerSec);
    }
    return Math.min(dashboardSpeedMetersPerSec, maxSpeedMetersPerSec);
  }

  public static int sanitizeTargetTagId(double dashboardTagId) {
    if (!Double.isFinite(dashboardTagId) || dashboardTagId < 0.0) {
      return Vision.ANY_HANDHELD_TAG_ID;
    }
    return (int) Math.round(dashboardTagId);
  }

  static Translation2d calculateTargetTranslation(
      Pose2d robotPose, HandheldTagObservation observation) {
    Rotation2d robotToTag =
        robotPose.getRotation().plus(observation.cameraYaw()).plus(observation.tx());
    Translation2d robotToTagTranslation =
        new Translation2d(observation.distanceToRobotMeters(), robotToTag);
    return robotPose.getTranslation().plus(robotToTagTranslation);
  }

  static Rotation2d calculateTagForwardDirection(
      Pose2d robotPose, Translation2d targetTranslation) {
    Translation2d robotToTarget = targetTranslation.minus(robotPose.getTranslation());
    if (robotToTarget.getNorm() <= 1e-9) {
      return robotPose.getRotation();
    }
    return robotToTarget.getAngle();
  }

  private void applySelectedBindingMode(DemoBindingMode bindingMode) {
    CommandScheduler.getInstance()
        .setActiveButtonLoop(
            switch (bindingMode) {
              case COMPETITION -> competitionButtonLoop;
              case CRAZY -> crazyButtonLoop;
            });

    appliedBindingMode = bindingMode;
    SmartDashboard.putString("Demo Mode/Bindings Applied", appliedBindingMode.name());
    Logger.recordOutput("Controls/DemoBindingModeApplied", appliedBindingMode.name());
  }

  private void publishSelectedBindingMode(DemoBindingMode selectedBindingMode) {
    if (selectedBindingMode == lastPublishedSelectedBindingMode) {
      return;
    }

    lastPublishedSelectedBindingMode = selectedBindingMode;
    SmartDashboard.putString("Demo Mode/Bindings Selected", selectedBindingMode.name());
    Logger.recordOutput("Controls/DemoBindingModeSelected", selectedBindingMode.name());
  }

  private DemoBindingMode getSelectedBindingMode() {
    DemoBindingMode selectedBindingMode = bindingModeChooser.get();
    return selectedBindingMode != null ? selectedBindingMode : DemoBindingMode.CRAZY;
  }

  private static void logTarget(boolean enabled, Translation2d target) {
    Logger.recordOutput("Controls/DemoHandheldTagTargetValid", true);
    Logger.recordOutput("Controls/DemoHandheldTagTargetingEnabled", enabled);
    Logger.recordOutput("Controls/DemoHandheldTagTargetX", target.getX());
    Logger.recordOutput("Controls/DemoHandheldTagTargetY", target.getY());
  }

  private void clearLoggedTarget(boolean enabled) {
    lastHandheldTarget = null;
    Logger.recordOutput("Controls/DemoHandheldTagTargetValid", false);
    Logger.recordOutput("Controls/DemoHandheldTagTargetingEnabled", enabled);
    Logger.recordOutput("Controls/DemoHandheldTagTargetX", 0.0);
    Logger.recordOutput("Controls/DemoHandheldTagTargetY", 0.0);
  }
}
