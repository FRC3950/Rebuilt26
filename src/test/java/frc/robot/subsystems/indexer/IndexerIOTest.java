package frc.robot.subsystems.indexer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IndexerIOTest {
  private static class Gate {
    boolean allowed = false;
  }

  private static class FakeIndexerIO implements IndexerIO {
    double indexerVelocityRps = 0.0;
    double hotdogVelocityRps = 0.0;
    double hotdogCurrentAmps = 0.0;

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
      inputs.indexerVelocityRps = indexerVelocityRps;
      inputs.hotdogVelocityRps = hotdogVelocityRps;
      inputs.hotdogCurrentAmps = hotdogCurrentAmps;
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

  @BeforeEach
  void pauseTiming() {
    SimHooks.pauseTiming();
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
  void latchedForwardFeedContinuesWhenGateDrops() {
    FakeIndexerIO io = new FakeIndexerIO();
    Gate gate = new Gate();
    gate.allowed = true;
    Indexer indexer = new Indexer(io, () -> gate.allowed);

    indexer.requestForwardFeed();

    assertTrue(indexer.isForwardFeedGateLatched());
    assertTrue(indexer.isFeedingForward());

    gate.allowed = false;
    indexer.periodic();

    assertTrue(indexer.isForwardFeedRequested());
    assertTrue(indexer.isForwardFeedGateLatched());
    assertTrue(indexer.isFeedingForward());
    assertEquals(
        frc.robot.Constants.SubsystemConstants.Indexer.indexerSpeed, io.indexerVelocityRps, 1e-9);
    assertEquals(
        frc.robot.Constants.SubsystemConstants.Indexer.hotdogSpeed, io.hotdogVelocityRps, 1e-9);
  }

  @Test
  void stoppingForwardFeedClearsGateLatchForNextRequest() {
    FakeIndexerIO io = new FakeIndexerIO();
    Gate gate = new Gate();
    gate.allowed = true;
    Indexer indexer = new Indexer(io, () -> gate.allowed);

    indexer.requestForwardFeed();
    assertTrue(indexer.isForwardFeedGateLatched());

    indexer.stopForwardFeed();
    gate.allowed = false;
    indexer.requestForwardFeed();

    assertTrue(indexer.isForwardFeedRequested());
    assertFalse(indexer.isForwardFeedGateLatched());
    assertFalse(indexer.isFeedingForward());
    assertEquals(0.0, io.indexerVelocityRps, 1e-9);
    assertEquals(0.0, io.hotdogVelocityRps, 1e-9);
  }

  @Test
  void manualCommandsClearForwardFeedGateLatch() {
    FakeIndexerIO io = new FakeIndexerIO();
    Gate gate = new Gate();
    gate.allowed = true;
    Indexer indexer = new Indexer(io, () -> gate.allowed);

    indexer.requestForwardFeed();
    assertTrue(indexer.isForwardFeedGateLatched());

    indexer.setIndexerSpeed(-frc.robot.Constants.SubsystemConstants.Indexer.indexerSpeed);
    gate.allowed = false;
    indexer.requestForwardFeed();

    assertTrue(indexer.isForwardFeedRequested());
    assertFalse(indexer.isForwardFeedGateLatched());
    assertFalse(indexer.isFeedingForward());
    assertEquals(0.0, io.indexerVelocityRps, 1e-9);
    assertEquals(0.0, io.hotdogVelocityRps, 1e-9);
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

  @Test
  void forwardHotdogStallCurrentStartsAutoUnjam() {
    FakeIndexerIO io = new FakeIndexerIO();
    Indexer indexer = new Indexer(io);

    indexer.startHotdog();
    io.hotdogCurrentAmps =
        frc.robot.Constants.SubsystemConstants.Indexer.autoUnjamHotdogStallCurrentAmps;
    indexer.periodic();

    assertTrue(indexer.isAutoUnjamActive());
    assertTrue(indexer.isHotdogStallCurrentExceeded());
    assertEquals(0.0, io.indexerVelocityRps, 1e-9);
    assertEquals(
        frc.robot.Constants.SubsystemConstants.Indexer.unjamHotdog, io.hotdogVelocityRps, 1e-9);
  }

  @Test
  void forwardFeedStallCurrentStopsIndexerWhileAutoUnjammingHotdog() {
    FakeIndexerIO io = new FakeIndexerIO();
    Indexer indexer = new Indexer(io, () -> true);

    indexer.requestForwardFeed();
    io.hotdogCurrentAmps =
        frc.robot.Constants.SubsystemConstants.Indexer.autoUnjamHotdogStallCurrentAmps;
    indexer.periodic();

    assertTrue(indexer.isAutoUnjamActive());
    assertTrue(indexer.isForwardFeedRequested());
    assertEquals(0.0, io.indexerVelocityRps, 1e-9);
    assertEquals(
        frc.robot.Constants.SubsystemConstants.Indexer.unjamHotdog, io.hotdogVelocityRps, 1e-9);
  }

  @Test
  void activeForwardFeedResumesAfterAutoUnjamWindow() {
    FakeIndexerIO io = new FakeIndexerIO();
    Indexer indexer = new Indexer(io, () -> true);

    indexer.requestForwardFeed();
    io.hotdogCurrentAmps =
        frc.robot.Constants.SubsystemConstants.Indexer.autoUnjamHotdogStallCurrentAmps;
    indexer.periodic();

    io.hotdogCurrentAmps = 0.0;
    SimHooks.stepTiming(
        frc.robot.Constants.SubsystemConstants.Indexer.autoUnjamReverseSeconds + 0.01);
    indexer.periodic();

    assertFalse(indexer.isAutoUnjamActive());
    assertTrue(indexer.isForwardFeedRequested());
    assertEquals(
        frc.robot.Constants.SubsystemConstants.Indexer.indexerSpeed, io.indexerVelocityRps, 1e-9);
    assertEquals(
        frc.robot.Constants.SubsystemConstants.Indexer.hotdogSpeed, io.hotdogVelocityRps, 1e-9);
  }

  @Test
  void stopCommandClearsActiveAutoUnjam() {
    FakeIndexerIO io = new FakeIndexerIO();
    Indexer indexer = new Indexer(io);

    indexer.startHotdog();
    io.hotdogCurrentAmps =
        frc.robot.Constants.SubsystemConstants.Indexer.autoUnjamHotdogStallCurrentAmps;
    indexer.periodic();

    indexer.stopHotdog();

    assertFalse(indexer.isAutoUnjamActive());
    assertEquals(0.0, io.indexerVelocityRps, 1e-9);
    assertEquals(0.0, io.hotdogVelocityRps, 1e-9);
  }

  @Test
  void reverseCommandClearsActiveAutoUnjam() {
    FakeIndexerIO io = new FakeIndexerIO();
    Indexer indexer = new Indexer(io);

    indexer.startHotdog();
    io.hotdogCurrentAmps =
        frc.robot.Constants.SubsystemConstants.Indexer.autoUnjamHotdogStallCurrentAmps;
    indexer.periodic();

    indexer.reverseHotdog();

    assertFalse(indexer.isAutoUnjamActive());
    assertEquals(0.0, io.indexerVelocityRps, 1e-9);
    assertEquals(
        frc.robot.Constants.SubsystemConstants.Indexer.unjamHotdog, io.hotdogVelocityRps, 1e-9);
  }

  @Test
  void belowThresholdCurrentDoesNotStartAutoUnjam() {
    FakeIndexerIO io = new FakeIndexerIO();
    Indexer indexer = new Indexer(io);

    indexer.startHotdog();
    io.hotdogCurrentAmps =
        frc.robot.Constants.SubsystemConstants.Indexer.autoUnjamHotdogStallCurrentAmps - 0.1;
    indexer.periodic();

    assertFalse(indexer.isAutoUnjamActive());
    assertFalse(indexer.isHotdogStallCurrentExceeded());
    assertEquals(0.0, io.indexerVelocityRps, 1e-9);
    assertEquals(
        frc.robot.Constants.SubsystemConstants.Indexer.hotdogSpeed, io.hotdogVelocityRps, 1e-9);
  }
}
