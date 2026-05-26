package frc.robot.subsystems.indexer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class IndexerIOTest {
  private static class Gate {
    boolean allowed = false;
  }

  private static class FakeIndexerIO implements IndexerIO {
    double indexerVelocityRps = 0.0;
    double hotdogVelocityRps = 0.0;

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
      inputs.indexerVelocityRps = indexerVelocityRps;
      inputs.hotdogVelocityRps = hotdogVelocityRps;
    }

    @Override
    public void setIndexerVelocity(double velocityRps) {
      indexerVelocityRps = velocityRps;
    }

    @Override
    public void setHotdogVelocity(double velocityRps) {
      hotdogVelocityRps = velocityRps;
    }
  }

  @Test
  void requestedForwardFeedIsBlockedUntilGateAllowsIt() {
    FakeIndexerIO io = new FakeIndexerIO();
    Gate gate = new Gate();
    Indexer indexer = new Indexer(io, () -> gate.allowed);

    indexer.requestForwardFeed();

    assertTrue(indexer.isForwardFeedRequested());
    assertFalse(indexer.isFeedingForward());
    assertEquals(0.0, io.indexerVelocityRps, 1e-9);
    assertEquals(0.0, io.hotdogVelocityRps, 1e-9);

    gate.allowed = true;
    indexer.periodic();

    assertTrue(indexer.isFeedingForward());
    assertEquals(
        frc.robot.Constants.SubsystemConstants.Indexer.indexerSpeed, io.indexerVelocityRps, 1e-9);
    assertEquals(
        frc.robot.Constants.SubsystemConstants.Indexer.hotdogSpeed, io.hotdogVelocityRps, 1e-9);
  }

  @Test
  void reverseAndSingleHotdogCommandsBypassForwardFeedGate() {
    FakeIndexerIO io = new FakeIndexerIO();
    Indexer indexer = new Indexer(io, () -> false);

    indexer.setIndexerSpeed(-frc.robot.Constants.SubsystemConstants.Indexer.indexerSpeed);
    indexer.setHotdogSpeed(-frc.robot.Constants.SubsystemConstants.Indexer.hotdogSpeed);

    assertEquals(
        -frc.robot.Constants.SubsystemConstants.Indexer.indexerSpeed, io.indexerVelocityRps, 1e-9);
    assertEquals(
        -frc.robot.Constants.SubsystemConstants.Indexer.hotdogSpeed, io.hotdogVelocityRps, 1e-9);

    indexer.stopIndexer();
    indexer.stopHotdog();
    indexer.startHotdog();

    assertEquals(0.0, io.indexerVelocityRps, 1e-9);
    assertEquals(
        frc.robot.Constants.SubsystemConstants.Indexer.hotdogSpeed, io.hotdogVelocityRps, 1e-9);
  }

  @Test
  void commandedAndMeasuredFeedingStateAreSeparate() {
    FakeIndexerIO io = new FakeIndexerIO();
    Indexer indexer = new Indexer(io);

    indexer.startIndexer();
    indexer.startHotdog();

    assertTrue(indexer.isFeedingForward());
    assertFalse(indexer.isMeasuredFeedingForward());

    indexer.periodic();

    assertTrue(indexer.isMeasuredFeedingForward());
  }
}
