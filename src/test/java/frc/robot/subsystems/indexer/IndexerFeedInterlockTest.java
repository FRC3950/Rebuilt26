package frc.robot.subsystems.indexer;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class IndexerFeedInterlockTest {
  @Test
  void forwardRequestPausesAndResumesWhenInterlockClears() {
    double requestedSpeed = 20.0;
    assertEquals(0.0, Indexer.permittedSpeed(requestedSpeed, false));
    assertEquals(requestedSpeed, Indexer.permittedSpeed(requestedSpeed, true));
  }

  @Test
  void reverseAndStopRemainAvailableWhileForwardFeedIsBlocked() {
    assertEquals(-20.0, Indexer.permittedSpeed(-20.0, false));
    assertEquals(0.0, Indexer.permittedSpeed(0.0, false));
  }
}
