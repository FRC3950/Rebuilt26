package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoChannel;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.turret.CRT;
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
      public static final int fLeftEncoderID = 3;
      public static final int fRightDriveID = 4;
      public static final int fRightSteerID = 5;
      public static final int fRightEncoderID = 6;
      public static final int bLeftDriveID = 7;
      public static final int bLeftSteerID = 8;
      public static final int bLeftEncoderID = 9;
      public static final int bRightDriveID = 10;
      public static final int bRightSteerID = 11;
      public static final int bRightEncoderID = 12;
    }

    public static final class Turret {
      public static final int azimuthID = 14;
      public static final int azimuthCanCoderAID = 15;
      public static final int azimuthCanCoderBID = 16;
      public static final int flywheelID = 17;
      public static final int flywheelFollowerID = 18;

      public static final int azimuthID2 = 19;
      public static final int azimuthCanCoderAID2 = 20;
      public static final int azimuthCanCoderBID2 = 21;
      public static final int flywheelID2 = 22;
      public static final int flywheelFollowerID2 = 23;

      public static final TalonFXConfiguration azimuthConfig = new TalonFXConfiguration();
      public static final TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

      // Gear Ratios (Placeholders - UPDATE ME)
      public static final double azimuthGearRatio = 0.1;
      public static final double flywheelGearRatio = 1.0;

      // Limits (Placeholders - UPDATE ME)
      public static final double minAzimuthAngle = -270.0;
      public static final double maxAzimuthAngle = 270.0;
      public static final double minHoodAngle = 10.0;
      public static final double maxHoodAngle = 50.0;

      // CRT constants are placeholders. Fill with real turret gearing/offset values.
      public static final int azimuthCrtModulusA = 20;
      public static final int azimuthCrtModulusB = 20;
      // In order to find the offset, you need to rotate the turret to the 0 degree position and
      // then read the rav value from the encoders in Phoenix Tuner.
      public static final double azimuthCanCoderAOffsetRot = 0.0;
      public static final double azimuthCanCoderBOffsetRot = 0.0;
      public static final double azimuthCrtZeroOffsetDeg = 0.0;

      public static final int azimuthCrtModulusA2 = 20;
      public static final int azimuthCrtModulusB2 = 20;
      // In order to find the offset, you need to rotate the turret to the 0 degree position and
      // then read the rav value from the encoders in Phoenix Tuner.
      public static final double azimuthCanCoderAOffsetRot2 = 0.0;
      public static final double azimuthCanCoderBOffsetRot2 = 0.0;
      public static final double azimuthCrtZeroOffsetDeg2 = 0.0;

      public static final CRT.Parameters azimuthCrtParams =
          new CRT.Parameters(
              azimuthCrtModulusA,
              azimuthCrtModulusB,
              azimuthCanCoderAOffsetRot,
              azimuthCanCoderBOffsetRot,
              azimuthCrtZeroOffsetDeg,
              minAzimuthAngle,
              maxAzimuthAngle);
      public static final CRT.Parameters azimuthCrtParams2 =
          new CRT.Parameters(
              azimuthCrtModulusA2,
              azimuthCrtModulusB2,
              azimuthCanCoderAOffsetRot2,
              azimuthCanCoderBOffsetRot2,
              azimuthCrtZeroOffsetDeg2,
              minAzimuthAngle,
              maxAzimuthAngle);

      public static final int HOOD_SERVO_HUB_CAN_ID = 1;
      public static final ServoChannel.ChannelId HOOD_SERVO_CHANNEL_1 =
          ServoChannel.ChannelId.kChannelId0;
      public static final ServoChannel.ChannelId HOOD_SERVO_CHANNEL_2 =
          ServoChannel.ChannelId.kChannelId1;
      public static final int HOOD_SERVO_MIN_PULSE_US = 500;
      public static final int HOOD_SERVO_CENTER_PULSE_US = 1500;
      public static final int HOOD_SERVO_MAX_PULSE_US = 2500;

      // PID / Motion Magic Gains (Placeholders - UPDATE ME)
      public static final double azimuthKP = 2.0;
      public static final double azimuthKD = 0.12;
      public static final double azimuthKS = 0.25;
      public static final double azimuthMaxVelocity = 100.0; // deg/s
      public static final double azimuthMaxAcceleration = 200.0; // deg/s^2

      public static final double flywheelKP = 0.2;
      public static final double flywheelKS = 0.0;
      public static final double flywheelKV = 0.11;

      // Turret position relative to the robot pose origin (meters).
      public static final Translation2d robotToTurret1 =
          new Translation2d(Units.inchesToMeters(-7.75), Units.inchesToMeters(-7.25));
      public static final Translation2d robotToTurret2 =
          new Translation2d(Units.inchesToMeters(7.75), Units.inchesToMeters(-7.25));

      // Extra latency to account for: vision delay + command pipeline + ball exit
      // time.
      // Tune this on-field until moving shots land consistently.
      // if everything doesnt work consistently, try adjusting this value
      public static final double shotExtraLatencySec = 0.08;

      static {
        // Azimuth Motor Config
        azimuthConfig.Slot0.kP = azimuthKP;
        azimuthConfig.Slot0.kD = azimuthKD;
        azimuthConfig.Slot0.kS = azimuthKS;
        azimuthConfig.MotionMagic.MotionMagicCruiseVelocity =
            azimuthMaxVelocity / 360.0 * azimuthGearRatio; // Rotations/s
        azimuthConfig.MotionMagic.MotionMagicAcceleration =
            azimuthMaxAcceleration / 360.0 * azimuthGearRatio; // Rotations/s^2
        azimuthConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Flywheel Motor Config
        flywheelConfig.Slot0.kP = flywheelKP;
        flywheelConfig.Slot0.kS = flywheelKS;
        flywheelConfig.Slot0.kV = flywheelKV;
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      }
    }

    public static final class Intake {
      public static final int intakeMotorID = 24;
      public static final int pivotMotorID = 25;

      // Intake Positions - UPDATE ME!!!
      public static final double downPos = -2.5;
      public static final double upPos = 2.5;

      public static final double intakeSpeed = 40.0; // RPS
      public static final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
      public static final double intakeKP = 0.1;
      public static final double intakeKV = 0.12;

      public static final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
      public static final double pivotKP = 0.2;
      public static final double pivotKV = 0.11;
      public static final int pivotMMVelocity = 25;
      public static final int pivotMMAcceleration = 10;

      static {
        pivotConfig.Slot0.kP = pivotKP;
        pivotConfig.Slot0.kV = pivotKV;
        pivotConfig.MotionMagic.MotionMagicAcceleration = pivotMMAcceleration;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = pivotMMVelocity;

        // Intake Motor Config
        intakeConfig.Slot0.kP = intakeKP;
        intakeConfig.Slot0.kV = intakeKV;
      }

      // for the pivot: motion magic with kv and kp, for intake: duty cycle.
    }

    public static final class Indexer {
      public static final int hotdogMotorID = 26;
      public static final int indexerMotorID = 27;

      public static final double indexerSpeed = 20.0; // RPS
      public static final double hotdogSpeed = 20.0; // RPS

      public static final TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
      public static final TalonFXConfiguration hotdogConfig = new TalonFXConfiguration();

      public static final double indexerKP = 0.1;
      public static final double indexerKV = 0.12;

      public static final double hotdogKP = 0.1;
      public static final double hotdogKV = 0.12;

      static {
        indexerConfig.Slot0.kP = indexerKP;
        indexerConfig.Slot0.kV = indexerKV;

        hotdogConfig.Slot0.kP = hotdogKP;
        hotdogConfig.Slot0.kV = hotdogKV;
      }
    }

    public static final class Climber {
      public static final int climberMotorID = 28;
      public static final int limitSwitchPort = 0;

      public static final double climberMaxHeight = 20;
      public static final double climbUpHeight = 18;
      public static final double climbFinalPos = 4;
      public static final double climbOffsetMeters = 0.50;
      public static final double climbHeadingDeg = -90.0;
      public static final double climberPositionTolerance = 0.25;
      public static final double climberMoveTimeoutSecs = 2.5;
      public static final double climbPathMaxVelocityMetersPerSec = 3.0;
      public static final double climbPathMaxAccelerationMetersPerSecSq = 3.0;
      public static final double climbPathMaxAngularVelocityDegPerSec = 540.0;
      public static final double climbPathMaxAngularAccelerationDegPerSecSq = 540.0;

      public static final TalonFXConfiguration climberConfig = new TalonFXConfiguration();
      public static final double climberKP = 0.2;
      public static final double climberKV = 0.11;
      public static final int climberMMVelocity = 25;
      public static final int climberMMAcceleration = 10;

      static {
        climberConfig.Slot0.kP = climberKP;
        climberConfig.Slot0.kV = climberKV;
        climberConfig.MotionMagic.MotionMagicAcceleration = climberMMAcceleration;
        climberConfig.MotionMagic.MotionMagicCruiseVelocity = climberMMVelocity;
        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = climberMaxHeight;
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
      }
    }
    // Josh wrote this part of the code, easter egg of 2026

    // Note: Sasha Isn't the person you ask
  }

  public static final class FieldConstants {
    // position of Hub
    public static final Translation2d hubTranslation =
        AllianceFlipUtil.apply(new Translation2d(4.6256194, 4.0346376));
    // Field dimensions in meters (blue-origin WPILib frame).
    public static final double fieldWidth = 8.0756125;
    public static final double fieldLength = 16.5411785;
    // positions of ferry shot targets
    // 1 foot = 0.3048 meters
    public static final Translation2d leftFerryTarget =
        AllianceFlipUtil.apply(
            new Translation2d(hubTranslation.getX() - 1.34, hubTranslation.getY() - 0.59));
    public static final Translation2d rightFerryTarget =
        AllianceFlipUtil.apply(
            new Translation2d(hubTranslation.getX() + 1.34, hubTranslation.getY() - 0.59));
    public static final Translation2d climbSouth =
        AllianceFlipUtil.apply(new Translation2d(1.06045, 3.29803125));

    public static final Translation2d climbNorth =
        AllianceFlipUtil.apply(climbSouth.plus(new Translation2d(0, 0.89535)));

    public static final double neutralZoneMinX = 5.1816;
    public static final double neutralZoneMaxX = 11.303;
    public static final double TRENCH_ALIGN_TIME_SEC = 0.40;
    public static final double TRENCH_DEBOUNCE_SEC = 0.10;

    public static final double scoreTime = 0.25;
  }
}
