package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.AllianceFlipUtil;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /** Main loop period used for simple derivatives/filters. */
  public static final double loopPeriodSecs = 0.02;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class SubsystemConstants {

    public static final CANBus CANivore = new CANBus("CANivore");

    public static final class Drive {
      public static final int fLeftDriveID = 1;
      public static final int fLeftSteerID = 2;
      public static final int fRightDriveID = 3;
      public static final int fRightSteerID = 4;
      public static final int bLeftDriveID = 5;
      public static final int bLeftSteerID = 6;
      public static final int bRightDriveID = 7;
      public static final int bRightSteerID = 8;
    }

    public static final class Turret {
      public static final int azimuthID = 10;
      public static final int hoodID = 11;
      public static final int flywheelID = 12;
      public static final int flywheelFollowerID = 13;

      public static final int azimuthID2 = 14;
      public static final int hoodID2 = 15;
      public static final int flywheelID2 = 16;
      public static final int flywheelFollowerID2 = 17;

      public static final TalonFXConfiguration azimuthConfig = new TalonFXConfiguration();
      public static final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
      public static final TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

      // Gear Ratios (Placeholders - UPDATE ME)
      public static final double azimuthGearRatio = 12.8;
      public static final double hoodGearRatio = 12.8;
      public static final double flywheelGearRatio = 1.0;

      // Limits (Placeholders - UPDATE ME)
      public static final double minAzimuthAngle = -180.0;
      public static final double maxAzimuthAngle = 180.0;
      public static final double minHoodAngle = 10.0;
      public static final double maxHoodAngle = 50.0;

      // PID / Motion Magic Gains (Placeholders - UPDATE ME)
      public static final double azimuthKP = 2.0;
      public static final double azimuthKV = 0.12;
      public static final double azimuthKS = 0.25;
      public static final double azimuthMaxVelocity = 100.0; // deg/s
      public static final double azimuthMaxAcceleration = 200.0; // deg/s^2

      public static final double hoodKP = 4.0;
      public static final double hoodKD = 0.5;

      public static final double flywheelKP = 0.2;
      public static final double flywheelKS = 0.0;
      public static final double flywheelKV = 0.11;

      // Turret position relative to the robot pose origin (meters).
      public static final Translation2d robotToTurret1 = new Translation2d(0.0, 0.0);
      public static final Translation2d robotToTurret2 = new Translation2d(0.0, 0.0);

      // Extra latency to account for: vision delay + command pipeline + ball exit
      // time.
      // Tune this on-field until moving shots land consistently.
      // if everything doesnt work consistently, try adjusting this value
      public static final double shotExtraLatencySec = 0.08;

      static {
        // Azimuth Motor Config
        azimuthConfig.Slot0.kP = azimuthKP;
        azimuthConfig.Slot0.kV = azimuthKV;
        azimuthConfig.Slot0.kS = azimuthKS;
        azimuthConfig.MotionMagic.MotionMagicCruiseVelocity =
            azimuthMaxVelocity / 360.0 * azimuthGearRatio; // Rotations/s
        azimuthConfig.MotionMagic.MotionMagicAcceleration =
            azimuthMaxAcceleration / 360.0 * azimuthGearRatio; // Rotations/s^2
        azimuthConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Hood Motor Config
        hoodConfig.Slot0.kP = hoodKP;
        hoodConfig.Slot0.kD = hoodKD;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Flywheel Motor Config
        flywheelConfig.Slot0.kP = flywheelKP;
        flywheelConfig.Slot0.kS = flywheelKS;
        flywheelConfig.Slot0.kV = flywheelKV;
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      }
    }
  }

  // Note: Sasha Isn't the person you ask
  public static final class FieldConstants {
    // position of Hub
    public static final Translation2d hubTranslation =
        AllianceFlipUtil.apply(new Translation2d(4.6256194, 4.0346376));
    // both in inches
    public static final double fieldWidth = 317.69;
    public static final double fieldLength = 651.2275;
    // positions of ferry shot targets
    // 1 foot = 0.3048 meters
    public static final Translation2d leftFerryTarget =
        AllianceFlipUtil.apply(
            new Translation2d(hubTranslation.getX() - 1.34, hubTranslation.getY() - 0.59));
    public static final Translation2d rightFerryTarget =
        AllianceFlipUtil.apply(
            new Translation2d(hubTranslation.getX() + 1.34, hubTranslation.getY() - 0.59));

    public static final double neutralZoneMinY = 5.1816;
    public static final double neutralZoneMaxY = 11.303;
  }
}
