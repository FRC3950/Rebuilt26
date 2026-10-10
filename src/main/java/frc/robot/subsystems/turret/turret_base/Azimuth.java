package frc.robot.subsystems.turret.turret_base;

import static frc.robot.Constants.SubsystemConstants.Turret.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CompletionException;

public class Azimuth {

  private final TalonFX azimuth;
  private final CANcoder encoder;
  private final double startupCenterDeg;
  private final double minAngleDeg;
  private final double maxAngleDeg;
  private boolean configurationApplied;
  private final PositionVoltage azimuthControl = new PositionVoltage(0.0);

  private double lastSetpointDeg = 0.0;
  private boolean hasCommandedSetpoint = false;
  private boolean startupReady = Constants.currentMode == Constants.Mode.SIM;
  private boolean verifyingStartup = false;
  private double startupSensorRotations = Double.NaN;
  private double verificationStartSec;
  private CompletableFuture<StatusCode> startupPositionWrite;
  private final StartupEnableRecovery enableRecovery = new StartupEnableRecovery();
  private String startupStatus = "Confirm both physical startup windows while disabled";

  public Azimuth(
      int azimuthID,
      TalonFXConfiguration azimuthConfig,
      CANBus canbus,
      CANcoder encoder,
      double startupCenterDeg,
      double minAngleDeg,
      double maxAngleDeg) {
    azimuth = new TalonFX(azimuthID, canbus);
    configurationApplied = azimuth.getConfigurator().apply(azimuthConfig).isOK();
    azimuth.hasResetOccurred();
    this.encoder = encoder;
    this.startupCenterDeg = startupCenterDeg;
    this.minAngleDeg = minAngleDeg;
    this.maxAngleDeg = maxAngleDeg;
  }

  public void setTargetAngleDeg(double targetAngleDeg) {
    if (!startupReady) {
      stop();
      return;
    }
    lastSetpointDeg = targetAngleDeg;
    hasCommandedSetpoint = true;
    double motorRotations = Units.degreesToRotations(targetAngleDeg) * azimuthGearRatio;
    azimuth.setControl(azimuthControl.withPosition(motorRotations));
  }

  public double getMotorAngleDeg() {
    double azimuthRotations = azimuth.getPosition().getValueAsDouble();
    return Units.rotationsToDegrees(azimuthRotations / azimuthGearRatio);
  }

  public double getMeasuredAngleDeg() {
    return getMotorAngleDeg();
  }

  public double getSetpointDeg() {
    return lastSetpointDeg;
  }

  /** Uses selected CANcoder feedback until the first target establishes command continuity. */
  public double getWrapReferenceAngleDeg() {
    if (!startupReady) {
      return Double.NaN;
    }
    if (hasCommandedSetpoint || Constants.currentMode == Constants.Mode.SIM) {
      return lastSetpointDeg;
    }
    // Use live selected feedback after startup verification, before the first target.
    var position = azimuth.getPosition().refresh();
    if (!position.getStatus().isOK()) {
      return Double.NaN;
    }
    return Units.rotationsToDegrees(position.getValueAsDouble() / azimuthGearRatio);
  }

  public void stop() {
    azimuth.stopMotor();
  }

  public boolean isStartupReady() {
    return startupReady;
  }

  public String getStartupStatus() {
    return startupReady ? "Ready" : startupStatus;
  }

  public double getStartupAngleDeg() {
    return Units.rotationsToDegrees(startupSensorRotations / azimuthGearRatio);
  }

  /** Operator confirmation supplies information that the absolute sensor cannot observe. */
  public void initializeFromStartupWindow() {
    if (!DriverStation.isDisabled() || Constants.currentMode != Constants.Mode.REAL) {
      return;
    }
    beginStartupInitialization();
  }

  private void beginStartupInitialization() {
    // An aborted attempt may still be finishing its bounded CAN call. Never overlap writes.
    if (startupPositionWrite != null && !startupPositionWrite.isDone()) {
      return;
    }
    failStartup("Initializing");
    if (!configurationApplied) {
      failStartup("Talon configuration failed; restart robot code and check CAN");
      return;
    }
    var absolute = encoder.getAbsolutePosition();
    var faults = encoder.getFaultField();
    if (!absolute.getStatus().isOK() || !faults.getStatus().isOK() || faults.getValue() != 0) {
      failStartup("CANcoder data invalid or fault present; inspect Tuner X");
      return;
    }
    startupSensorRotations =
        StartupWindow.selectSensorRotations(
            absolute.getValueAsDouble(),
            startupCenterDeg,
            azimuthStartupHalfWindowDeg,
            azimuthGearRatio,
            minAngleDeg,
            maxAngleDeg);
    if (!Double.isFinite(startupSensorRotations)) {
      failStartup("Reading outside startup window or invalid window configuration");
      return;
    }
    // Consume previous resets. Any new reset during verification invalidates this attempt.
    encoder.hasResetOccurred();
    // Assign the reconstructed turn count, preserving the calibrated absolute/magnet offset.
    // Phoenix rejects a zero timeout. Run its bounded blocking write off the scheduler thread.
    startupPositionWrite =
        StartupPositionWrite.submit(
            (position, timeout) -> encoder.setPosition(position, timeout),
            startupSensorRotations,
            azimuthStartupPositionWriteTimeoutSec);
    verifyingStartup = true;
    startupStatus = "Assigning CANcoder position";
  }

  public void updateStartup(boolean automaticRecoveryAllowed) {
    if (Constants.currentMode != Constants.Mode.REAL) {
      return;
    }
    boolean recoverOnEnable =
        enableRecovery.shouldAttempt(
            DriverStation.isEnabled(), startupReady, verifyingStartup, automaticRecoveryAllowed);
    boolean encoderReset = encoder.hasResetOccurred();
    boolean motorReset = azimuth.hasResetOccurred();
    if (motorReset) {
      configurationApplied = false;
      failStartup("Talon reset; restart robot code to reapply configuration, then confirm windows");
      return;
    }
    if (encoderReset) {
      failStartup("Device reset; disable, return to windows, and confirm again");
      return;
    }
    // Placement in the marked windows is still required. Reuse the manual checks once at enable.
    if (recoverOnEnable) {
      beginStartupInitialization();
    }
    if (!startupReady && !verifyingStartup) {
      return;
    }
    if (verifyingStartup && startupPositionWrite != null) {
      if (!startupPositionWrite.isDone()) {
        return;
      }
      StatusCode writeStatus;
      try {
        writeStatus = startupPositionWrite.join();
      } catch (CompletionException exception) {
        startupPositionWrite = null;
        failStartup("CANcoder position assignment threw: " + exception.getCause());
        return;
      }
      startupPositionWrite = null;
      if (!writeStatus.isOK()) {
        failStartup(
            "CANcoder position assignment failed: "
                + writeStatus.getName()
                + " - "
                + writeStatus.getDescription());
        return;
      }
      verificationStartSec = Timer.getFPGATimestamp();
      startupStatus = "Verifying CANcoder and Talon feedback";
    }
    var sensorPosition = encoder.getPosition();
    var motorPosition = azimuth.getPosition();
    var faults = encoder.getFaultField();
    var remoteInvalid = azimuth.getFault_RemoteSensorDataInvalid();
    boolean valid =
        sensorPosition.getStatus().isOK()
            && motorPosition.getStatus().isOK()
            && faults.getStatus().isOK()
            && faults.getValue() == 0
            && remoteInvalid.getStatus().isOK()
            && !remoteInvalid.getValue()
            && Double.isFinite(sensorPosition.getValueAsDouble())
            && Double.isFinite(motorPosition.getValueAsDouble());
    double toleranceRotations = azimuthStartupVerificationToleranceDeg / 360.0 * azimuthGearRatio;
    if (startupReady) {
      if (!valid) {
        failStartup("Feedback invalid; disable and inspect before reinitializing");
      }
      return;
    }
    double elapsed = Timer.getFPGATimestamp() - verificationStartSec;
    if (elapsed > azimuthStartupVerificationTimeoutSec) {
      failStartup("Feedback verification timed out; inspect CANcoder and Talon in Tuner X");
    } else if (elapsed >= azimuthStartupVerificationDelaySec
        && valid
        && Math.abs(sensorPosition.getValueAsDouble() - startupSensorRotations)
            <= toleranceRotations
        && Math.abs(motorPosition.getValueAsDouble() - startupSensorRotations)
            <= toleranceRotations) {
      lastSetpointDeg =
          Units.rotationsToDegrees(motorPosition.getValueAsDouble() / azimuthGearRatio);
      hasCommandedSetpoint = false;
      verifyingStartup = false;
      startupReady = true;
    }
  }

  private void failStartup(String reason) {
    startupReady = false;
    verifyingStartup = false;
    hasCommandedSetpoint = false;
    startupStatus = reason;
    stop();
  }
}
