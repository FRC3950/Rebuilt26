package frc.robot.sim;

import static frc.robot.Constants.SubsystemConstants.Intake.downPos;
import static frc.robot.Constants.SubsystemConstants.Intake.upPos;
import static frc.robot.Constants.SubsystemConstants.Turret.maxHoodAngle;
import static frc.robot.Constants.SubsystemConstants.Turret.minHoodAngle;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class MechanismSimConstants {
  private MechanismSimConstants() {}

  public static final double LOOP_PERIOD_SECS = 0.02;
  public static final double MAX_VOLTAGE = 12.0;

  public static final class Intake {
    private Intake() {}

    public static final DCMotor ROLLER_GEARBOX = DCMotor.getKrakenX44(1);
    public static final double ROLLER_GEAR_RATIO = 1.0;
    public static final double ROLLER_MOI_KG_METERS_SQUARED = 0.0015;
    public static final double ROLLER_KP = 0.35;
    public static final double ROLLER_KD = 0.0;
    public static final double ROLLER_KV_VOLTS_PER_RPS = 0.105;

    public static final DCMotor PIVOT_GEARBOX = DCMotor.getKrakenX44(1);
    public static final double PIVOT_GEAR_RATIO = 45.0;
    public static final double PIVOT_LENGTH_METERS = Units.inchesToMeters(13.0);
    public static final double PIVOT_MASS_KG = 4.0;
    public static final double PIVOT_MIN_ANGLE_RAD = Units.degreesToRadians(0.0);
    public static final double PIVOT_MAX_ANGLE_RAD = Units.degreesToRadians(85.0);
    public static final double PIVOT_STARTING_ANGLE_RAD = PIVOT_MIN_ANGLE_RAD;
    public static final double PIVOT_KP = 8.0;
    public static final double PIVOT_KD = 0.0;
    public static final double PIVOT_KG_VOLTS = 0.4;
    public static final double PIVOT_POSITION_TOLERANCE = 0.05;

    public static double pivotPositionToAngleRad(double position) {
      double t = (position - upPos) / (downPos - upPos);
      return PIVOT_MIN_ANGLE_RAD + t * (PIVOT_MAX_ANGLE_RAD - PIVOT_MIN_ANGLE_RAD);
    }

    public static double pivotAngleRadToPosition(double angleRad) {
      double t = (angleRad - PIVOT_MIN_ANGLE_RAD) / (PIVOT_MAX_ANGLE_RAD - PIVOT_MIN_ANGLE_RAD);
      return upPos + t * (downPos - upPos);
    }
  }

  public static final class Indexer {
    private Indexer() {}

    public static final DCMotor INDEXER_GEARBOX = DCMotor.getKrakenX60Foc(1);
    public static final double INDEXER_GEAR_RATIO = 1.0;
    public static final double INDEXER_MOI_KG_METERS_SQUARED = 0.001;
    public static final double INDEXER_KP = 0.08;
    public static final double INDEXER_KD = 0.0;
    public static final double INDEXER_KV_VOLTS_PER_RPS = 0.1285;

    public static final DCMotor HOTDOG_GEARBOX = DCMotor.getKrakenX44(1);
    public static final double HOTDOG_GEAR_RATIO = 1.0;
    public static final double HOTDOG_MOI_KG_METERS_SQUARED = 0.001;
    public static final double HOTDOG_KP = 0.08;
    public static final double HOTDOG_KD = 0.0;
    public static final double HOTDOG_KV_VOLTS_PER_RPS = 0.11;
  }

  public static final class Turret {
    private Turret() {}

    public static final DCMotor AZIMUTH_GEARBOX = DCMotor.getKrakenX44(1);
    public static final double AZIMUTH_MOI_KG_METERS_SQUARED = 0.02;
    public static final double AZIMUTH_KP = 5.0;
    public static final double AZIMUTH_KD = 0.1;

    public static final DCMotor FLYWHEEL_GEARBOX = DCMotor.getKrakenX44(1);
    public static final double FLYWHEEL_MOI_KG_METERS_SQUARED = 0.0008;
    public static final double FLYWHEEL_KP = 1.2;
    public static final double FLYWHEEL_KD = 0.25;
    public static final double FLYWHEEL_KV_VOLTS_PER_RPS = 0.0925;
    public static final double FLYWHEEL_VOLTAGE_RAMP_PERIOD_SECS = 0.1;

    public static final double HOOD_MIN_ANGLE_DEG = minHoodAngle;
    public static final double HOOD_MAX_ANGLE_DEG = maxHoodAngle;
    public static final double HOOD_MAX_VELOCITY_DEG_PER_SEC = 120.0;
  }
}
