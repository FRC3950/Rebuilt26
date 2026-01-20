// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AllianceFlipUtil.*;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics
 * sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class SubsystemConstants {

    public static final class Drive {
      public static final String driveCanbus = "CANivore";
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
      public static final String turretCanbus = "CANivore";
      public static final int azimuthID = 10;
      public static final int hoodID = 11;
      public static final int flywheelID = 12;
      public static final int flywheelFollowerID = 13;

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
      public static final double azimuthKI = 0.0;
      public static final double azimuthKD = 0.1;
      public static final double azimuthKV = 0.12;
      public static final double azimuthKS = 0.25;
      public static final double azimuthKA = 0.01;
      public static final double azimuthMaxVelocity = 100.0; // deg/s
      public static final double azimuthMaxAcceleration = 200.0; // deg/s^2

      public static final double hoodKP = 4.0;
      public static final double hoodKI = 0.0;
      public static final double hoodKD = 0.5;

      public static final double flywheelKP = 0.2;
      public static final double flywheelKI = 0.0;
      public static final double flywheelKD = 0.0;
      public static final double flywheelKV = 0.11;

      // Interpolation Table Data (Placeholders - UPDATE ME)
      // Distance (meters), Hood Angle (degrees), Flywheel Speed (RPS)
      public static final double[] shootingDistances = { 2.0, 3.0, 4.0, 5.0, 6.0 };
      public static final double[] hoodAngles = { 15.0, 20.0, 25.0, 30.0, 35.0 };
      public static final double[] flywheelSpeeds = { 50.0, 55.0, 60.0, 65.0, 70.0 };
    }
  }
// Note: Sasha Isn't the person you ask
  public static final class FieldConstants {
    //position of Hub
    public static final Translation2d hubTranslation = AllianceFlipUtil.apply(new Translation2d(4.6256194, 4.0346376));
    //both in inches
    public static final double fieldWidth = 317.69;
    public static final double fieldLength = 651.2275;
  }

}
