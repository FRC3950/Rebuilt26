package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  class IndexerIOInputs {
    public boolean indexerConnected = false;
    public double indexerVelocityRps = 0.0;
    public double indexerAppliedVolts = 0.0;
    public double indexerCurrentAmps = 0.0;

    public boolean hotdogConnected = false;
    public double hotdogVelocityRps = 0.0;
    public double hotdogAppliedVolts = 0.0;
    public double hotdogCurrentAmps = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void setIndexerVelocity(double velocityRps) {}

  default void setHotdogVelocity(double velocityRps) {}
}
