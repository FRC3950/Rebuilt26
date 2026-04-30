package frc.robot.subsystems.indexer;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class IndexerIOTest {
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
