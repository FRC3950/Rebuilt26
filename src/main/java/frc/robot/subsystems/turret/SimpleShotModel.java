package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.SubsystemConstants.Turret;

/** Dashboard calibration and telemetry for PrecomputedShotMap. */
public final class SimpleShotModel {
  public static final String ROOT = "ShotModel/";
  // Starting field/ball values from FuelSim, not new hardware measurements.
  public static final double HUB_HEIGHT = 1.83;

  public enum Param {
    HEIGHT("Setup/Release Height M", Constants.SimConstants.Fuel.TURRET_LAUNCH_HEIGHT_METERS),
    RADIUS("Setup/Wheel Radius M", Constants.SimConstants.Fuel.EFFECTIVE_FLYWHEEL_RADIUS_METERS),
    ELEV_MIN("Tune/Elevation At Hood Min Deg", 90 - Turret.minHoodAngle),
    ELEV_MAX("Tune/Elevation At Hood Max Deg", 90 - Turret.maxHoodAngle),
    TRANSFER("Tune/Ball Speed Transfer", 1),
    DRAG("Advanced/Drag Cd", 0),
    SPIN_RATIO("Advanced/Ball Spin Per Wheel Rev", 0),
    LIFT("Advanced/Magnus Coefficient", 0),
    SPIN_DECAY("Advanced/Spin Decay Per Sec", 0),
    DELAY("Advanced/Release Delay Sec", 0),
    LEFT_SPEED("Left/Speed Multiplier", 1),
    RIGHT_SPEED("Right/Speed Multiplier", 1),
    LEFT_HOOD("Left/Hood Trim Deg", 0),
    RIGHT_HOOD("Right/Hood Trim Deg", 0),
    LEFT_YAW("Left/Aim Trim Deg", 0),
    RIGHT_YAW("Right/Aim Trim Deg", 0);

    public final String key;
    public final double initial;

    Param(String key, double initial) {
      this.key = ROOT + key;
      this.initial = initial;
    }
  }

  public static final class Settings {
    private final double[] values;

    private Settings(double[] values) {
      this.values = values.clone();
    }

    public double get(Param key) {
      return values[key.ordinal()];
    }

    public Settings with(Param key, double value) {
      double[] next = values.clone();
      next[key.ordinal()] = value;
      return new Settings(next);
    }

    public String error() {
      for (double value : values)
        if (!Double.isFinite(value)) return "A tuning value is not finite";
      if (get(Param.HEIGHT) <= 0 || get(Param.HEIGHT) >= HUB_HEIGHT)
        return "Release height must be between floor and hub opening";
      if (get(Param.RADIUS) <= 0 || get(Param.RADIUS) > .3) return "Check wheel radius in meters";
      if (get(Param.TRANSFER) <= 0 || get(Param.TRANSFER) > 2)
        return "Ball Speed Transfer must be >0 and <=2";
      if (get(Param.ELEV_MIN) < 5
          || get(Param.ELEV_MIN) > 88
          || get(Param.ELEV_MAX) < 5
          || get(Param.ELEV_MAX) > 88
          || Math.abs(get(Param.ELEV_MAX) - get(Param.ELEV_MIN)) < 1)
        return "Set two distinct physical launch elevations between 5 and 88 degrees";
      if (get(Param.DRAG) < 0
          || get(Param.DRAG) > 2
          || get(Param.LIFT) < 0
          || get(Param.LIFT) > 2
          || get(Param.SPIN_DECAY) < 0
          || get(Param.SPIN_DECAY) > 10
          || Math.abs(get(Param.SPIN_RATIO)) > 10) return "Aerodynamic corrections out of bounds";
      if (get(Param.DELAY) < 0 || get(Param.DELAY) > .5)
        return "Release delay must be 0 to 0.5 seconds";
      if (get(Param.LEFT_SPEED) <= 0
          || get(Param.LEFT_SPEED) > 2
          || get(Param.RIGHT_SPEED) <= 0
          || get(Param.RIGHT_SPEED) > 2) return "Side speed multipliers must be >0 and <=2";
      return "";
    }
  }

  public static Settings defaults() {
    double[] values = new double[Param.values().length];
    for (Param p : Param.values()) values[p.ordinal()] = p.initial;
    return new Settings(values);
  }

  private static Settings current = defaults();

  /** Called by Tune, not Robot.java. Corrections are neutral unless the operator changes them. */
  static void initializeDashboard() {
    PrecomputedShotMap.initializeDashboard();
    double[] values = new double[Param.values().length];
    for (Param p : Param.values()) {
      values[p.ordinal()] = Preferences.getDouble(p.key, p.initial);
      SmartDashboard.putNumber(p.key, values[p.ordinal()]);
    }
    current = new Settings(values);
    SmartDashboard.putBoolean(ROOT + "Save Settings", false);
    SmartDashboard.putBoolean(ROOT + "Reset To Baseline", false);
    SmartDashboard.putString(
        ROOT + "Save Status",
        "Map inputs require offline regeneration; Save Settings persists dashboard calibration");
    SmartDashboard.putString(
        ROOT + "Baseline Note",
        "Angle mapping and wheel transfer are ideal assumptions. Drag, spin/lift and delay start disabled.");
    SmartDashboard.putNumber(ROOT + "Known/Hub Height M", HUB_HEIGHT);
    SmartDashboard.putNumber(ROOT + "Known/Hood Min Deg", Turret.minHoodAngle);
    SmartDashboard.putNumber(ROOT + "Known/Hood Max Deg", Turret.maxHoodAngle);
    SmartDashboard.putNumber(ROOT + "Known/Max Wheel RPS", Turret.maxFlywheelRps);
    SmartDashboard.putNumber(ROOT + "Known/Left Turret X M", Turret.robotToTurret1.getX());
    SmartDashboard.putNumber(ROOT + "Known/Left Turret Y M", Turret.robotToTurret1.getY());
    SmartDashboard.putNumber(ROOT + "Known/Right Turret X M", Turret.robotToTurret2.getX());
    SmartDashboard.putNumber(ROOT + "Known/Right Turret Y M", Turret.robotToTurret2.getY());
    refreshDashboard();
  }

  static void refreshDashboard() {
    if (SmartDashboard.getBoolean(ROOT + "Reset To Baseline", false)) {
      for (Param p : Param.values()) SmartDashboard.putNumber(p.key, p.initial);
      SmartDashboard.putBoolean(ROOT + "Reset To Baseline", false);
      SmartDashboard.putString(
          ROOT + "Save Status",
          "Baseline restored live; saved settings unchanged until Save Settings");
    }
    double[] values = new double[Param.values().length];
    for (Param p : Param.values()) values[p.ordinal()] = SmartDashboard.getNumber(p.key, p.initial);
    current = new Settings(values);
    String error = current.error();
    SmartDashboard.putBoolean(ROOT + "Settings Valid", error.isEmpty());
    SmartDashboard.putString(
        ROOT + "Settings Status",
        error.isEmpty() ? "Precomputed map mode - calibration is operator-verified" : error);
    if (SmartDashboard.getBoolean(ROOT + "Save Settings", false)) {
      SmartDashboard.putBoolean(ROOT + "Save Settings", false);
      if (error.isEmpty()) {
        for (Param p : Param.values()) Preferences.setDouble(p.key, current.get(p));
        SmartDashboard.putString(ROOT + "Save Status", "Saved to robot Preferences");
      } else SmartDashboard.putString(ROOT + "Save Status", "Not saved: " + error);
    }
  }

  public record Solution(
      GetAdjustedShot.ShootingParameters parameters,
      double distance,
      double radial,
      double tangential,
      double timeOfFlight) {}

  public static Solution solve(
      Pose2d pose,
      ChassisSpeeds velocity,
      Translation2d target,
      Translation2d offset,
      double roll,
      double pitch) {
    Solution result =
        PrecomputedShotMap.solve(current, pose, velocity, target, offset, roll, pitch);
    String side = offset.getY() >= 0 ? "Left" : "Right", key = ROOT + side + "/";
    var p = result.parameters();
    SmartDashboard.putBoolean(key + "Model Valid", p.isValid());
    SmartDashboard.putString(
        key + "Model Status",
        p.isValid()
            ? "Precomputed radial quadratic + 3D motion compensation; physical calibration unverified"
            : p.invalidReason());
    SmartDashboard.putNumber(key + "Distance M", result.distance());
    SmartDashboard.putNumber(key + "Radial MPS", result.radial());
    SmartDashboard.putNumber(key + "Tangential MPS", result.tangential());
    SmartDashboard.putNumber(key + "Flight Time Sec", result.timeOfFlight());
    SmartDashboard.putNumber(key + "Calculated Hood Deg", p.hoodAngleDeg());
    SmartDashboard.putNumber(key + "Calculated Wheel RPS", p.flywheelSpeed());
    return result;
  }

  public static double hoodForElevation(Settings c, double elevation) {
    return Turret.minHoodAngle
        + (elevation - c.get(Param.ELEV_MIN))
            * (Turret.maxHoodAngle - Turret.minHoodAngle)
            / (c.get(Param.ELEV_MAX) - c.get(Param.ELEV_MIN));
  }

  public static double wheelRps(Settings c, double speed, boolean left) {
    return speed
        / (2 * Math.PI * c.get(Param.RADIUS) * c.get(Param.TRANSFER))
        * c.get(left ? Param.LEFT_SPEED : Param.RIGHT_SPEED);
  }
}
