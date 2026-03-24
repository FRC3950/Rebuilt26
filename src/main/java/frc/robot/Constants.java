package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoChannel;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
      public static final int fLeftDriveID = 9;
      public static final int fLeftSteerID = 8;
      public static final int fLeftEncoderID = 44;
      public static final int fRightDriveID = 6;
      public static final int fRightSteerID = 7;
      public static final int fRightEncoderID = 41;
      public static final int bLeftDriveID = 0;
      public static final int bLeftSteerID = 1;
      public static final int bLeftEncoderID = 43;
      public static final int bRightDriveID = 10;
      public static final int bRightSteerID = 11;
      public static final int bRightEncoderID = 42;

      public static final int pigeonID = 46;
    }

    public static final class Turret {
      public static final int azimuthID = 19;
      public static final int TURRET_CANDI_ID = 45;
      public static final int flywheelID = 3;
      public static final int flywheelFollowerID = 2;

      public static final int azimuthID2 = 17;
      public static final int flywheelID2 = 16;
      public static final int flywheelFollowerID2 = 18;

      public static final TalonFXConfiguration leftAzimuthConfig = new TalonFXConfiguration();
      public static final TalonFXConfiguration rightAzimuthConfig = new TalonFXConfiguration();
      public static final TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

      public static final double azimuthGearRatio = 10;
      public static final double flywheelGearRatio = 1.0;

      // Limits (Placeholders - UPDATE ME)
      public static final double leftMinAzimuthControlAngle = -330;
      public static final double leftMaxAzimuthControlAngle = 10;
      public static final double rightMinAzimuthControlAngle = -330;
      public static final double rightMaxAzimuthControlAngle = 10;
      public static final double azimuthSoftLimitMarginDeg = 10.0;
      public static final double minHoodAngle = 13;
      public static final double maxHoodAngle = 29.85;
      public static final double minFlywheelRps = 0.0;
      public static final double maxFlywheelRps = 100.0;
      public static final double TURRET_LIMIT_SWITCH_ANGLE_DEG = 0;

      public static final int HOOD_SERVO_HUB_CAN_ID = 63;
      public static final ServoChannel.ChannelId HOOD_SERVO_CHANNEL_1 =
          ServoChannel.ChannelId.kChannelId0;
      public static final ServoChannel.ChannelId HOOD_SERVO_CHANNEL_2 =
          ServoChannel.ChannelId.kChannelId1;
      public static final int HOOD_SERVO_MIN_PULSE_US = 500;
      public static final int HOOD_SERVO_CENTER_PULSE_US = 1500;
      public static final int HOOD_SERVO_MAX_PULSE_US = 2500;

      // PID / Motion Magic Gains
      public static final double azimuthKP = 50;
      public static final double azimuthKS = 0;
      public static final double azimuthKV = 0.12;
      public static final double azimuthMMVelocity = 24;
      public static final double azimuthMMAcceleration = 48;

      public static final double flywheelKP = 1.2;
      public static final double flywheelKD = 0.25;
      public static final double flywheelKV = 0.0925;

      public static final Translation2d robotToTurret1 =
          new Translation2d(Units.inchesToMeters(-7.25), Units.inchesToMeters(7.75));
      public static final Translation2d robotToTurret2 =
          new Translation2d(Units.inchesToMeters(-7.25), Units.inchesToMeters(-7.75));

      // hi cj love you -ruby
      // where where t-square?
      static {
        // Azimuth Motor Config
        applyAzimuthConfig(
            leftAzimuthConfig,
            ForwardLimitSourceValue.RemoteCANdiS1,
            leftMinAzimuthControlAngle,
            leftMaxAzimuthControlAngle);
        applyAzimuthConfig(
            rightAzimuthConfig,
            ForwardLimitSourceValue.RemoteCANdiS2,
            rightMinAzimuthControlAngle,
            rightMaxAzimuthControlAngle);

        // Flywheel Motor Config
        flywheelConfig.Slot0.kP = flywheelKP;
        flywheelConfig.Slot0.kD = flywheelKD;
        flywheelConfig.Slot0.kV = flywheelKV;
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .1;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = 120;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = 80;
      }

      private static void applyAzimuthConfig(
          TalonFXConfiguration config,
          ForwardLimitSourceValue forwardLimitSource,
          double minControlAngleDeg,
          double maxControlAngleDeg) {
        config.Slot0.kP = azimuthKP;
        config.Slot0.kV = azimuthKV;
        config.Slot0.kS = azimuthKS;
        config.MotionMagic.MotionMagicCruiseVelocity = azimuthMMVelocity;
        config.MotionMagic.MotionMagicAcceleration = azimuthMMAcceleration;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0;
        config.HardwareLimitSwitch.ForwardLimitRemoteSensorID = TURRET_CANDI_ID;
        config.HardwareLimitSwitch.ForwardLimitSource = forwardLimitSource;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            Units.degreesToRotations(
                    maxControlAngleDeg + azimuthSoftLimitMarginDeg - TURRET_LIMIT_SWITCH_ANGLE_DEG)
                * azimuthGearRatio;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            Units.degreesToRotations(
                    minControlAngleDeg - azimuthSoftLimitMarginDeg - TURRET_LIMIT_SWITCH_ANGLE_DEG)
                * azimuthGearRatio;
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.SupplyCurrentLimit = 30;
      }
    }

    public static final class Intake {
      public static final int intakeMotorID = 5;
      public static final int pivotMotorID = 12;

      // Intake Positions - UPDATE ME!!!
      public static final double downPos = 19;
      public static final double upPos = 0.2;

      public static final double mintakeSpeed = 40;
      public static final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
      public static final double intakeKP = 0.1;
      public static final double intakeKV = 0.12;

      public static final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
      public static final double pivotKP = 15;
      public static final double pivotKS = 0.5;
      public static final double pivotKV = 0.12;
      public static final double pivotKG = 1;
      public static final int pivotMMVelocity = 60;
      public static final int pivotMMAcceleration = 120;

      static {
        pivotConfig.Slot0.kP = pivotKP;
        pivotConfig.Slot0.kV = pivotKV;
        pivotConfig.Slot0.kS = pivotKS;
        pivotConfig.Slot0.kG = pivotKG;
        pivotConfig.MotionMagic.MotionMagicAcceleration = pivotMMAcceleration;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = pivotMMVelocity;
        pivotConfig.CurrentLimits.StatorCurrentLimit = 80;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 40;

        // Intake Motor Config
        intakeConfig.Slot0.kP = intakeKP;
        intakeConfig.Slot0.kV = intakeKV;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.CurrentLimits.StatorCurrentLimit = 80;
      }
    }

    public static final class Indexer {
      public static final int hotdogMotorID = 4;
      public static final int indexerMotorID = 14;

      public static final double indexerSpeed = 50.0; // RPS
      public static final double hotdogSpeed = 40.0; // RPS

      public static final TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
      public static final TalonFXConfiguration hotdogConfig = new TalonFXConfiguration();

      public static final double indexerKP = 0;
      public static final double indexerKV = 0.1285;

      public static final double hotdogKP = 0;
      public static final double hotdogKV = 0.1175;

      static {
        indexerConfig.Slot0.kP = indexerKP;
        indexerConfig.Slot0.kV = indexerKV;
        indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        indexerConfig.CurrentLimits.StatorCurrentLimit = 120;
        indexerConfig.CurrentLimits.SupplyCurrentLimit = 70;

        hotdogConfig.Slot0.kP = hotdogKP;
        hotdogConfig.Slot0.kV = hotdogKV;
        hotdogConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hotdogConfig.CurrentLimits.StatorCurrentLimit = 80;
      }
    }
    // Josh wrote this part of the code, easter egg of 2026

    // Note: Sasha Isn't the person you ask
  }

  public static final class FieldConstants {
    // position of Hub
    public static final Translation2d hubGuy = new Translation2d(4.6256194, 4.0346376);

    public static final Translation2d hubTranslation = AllianceFlipUtil.apply(hubGuy);
    // Field dimensions in meters (blue-origin WPILib frame).
    public static final double fieldWidth = 8.0756125;
    public static final double fieldLength = 16.5411785;
    // positions of ferry shot targets
    // 1 foot = 0.3048 meters

    //
    public static final Translation2d leftFerryTarget =
        AllianceFlipUtil.apply(new Translation2d(hubGuy.getX() - 1.5, hubGuy.getY() + 2));
    public static final Translation2d rightFerryTarget =
        AllianceFlipUtil.apply(new Translation2d(hubGuy.getX() - 1.5, hubGuy.getY() - 2));

    public static final double neutralZoneMinX = 5.1816;
    public static final double neutralZoneMaxX = 11.303;
    public static final double TRENCH_ALIGN_TIME_SEC = 0.40;
    public static final double TRENCH_DEBOUNCE_SEC = 0.10;

    public static final double scoreTime = 0.25;

    public static boolean isRobotInNeutralZone(double robotXMeters) {
      return robotXMeters >= neutralZoneMinX && robotXMeters <= neutralZoneMaxX;
    }

    public static Translation2d getCloserFerryTarget(Translation2d robotTranslation) {
      return robotTranslation.getDistance(leftFerryTarget)
              <= robotTranslation.getDistance(rightFerryTarget)
          ? leftFerryTarget
          : rightFerryTarget;
    }
  }
}
