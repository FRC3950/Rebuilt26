package frc.robot.subsystems.turret.turret_base;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.StatusCode;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class StartupPositionWriteTest {
  @Test
  void submitsPositiveTimeoutWithoutWaitingForTheWrite() throws Exception {
    CountDownLatch started = new CountDownLatch(1);
    CountDownLatch release = new CountDownLatch(1);
    AtomicReference<Double> writtenPosition = new AtomicReference<>();
    AtomicReference<Double> writtenTimeout = new AtomicReference<>();
    var future =
        StartupPositionWrite.submit(
            (position, timeout) -> {
              writtenPosition.set(position);
              writtenTimeout.set(timeout);
              started.countDown();
              try {
                if (!release.await(2, TimeUnit.SECONDS)) {
                  return StatusCode.RxTimeout;
                }
              } catch (InterruptedException exception) {
                Thread.currentThread().interrupt();
                return StatusCode.RxTimeout;
              }
              return StatusCode.OK;
            },
            -5.0,
            0.1);
    try {
      assertTrue(started.await(1, TimeUnit.SECONDS));
      assertFalse(future.isDone());
      assertEquals(-5.0, writtenPosition.get());
      assertEquals(0.1, writtenTimeout.get());
    } finally {
      release.countDown();
    }
    assertEquals(StatusCode.OK, future.get(1, TimeUnit.SECONDS));
  }

  @Test
  void rejectsZeroTimeoutBeforeCallingPhoenix() {
    assertThrows(
        IllegalArgumentException.class,
        () -> StartupPositionWrite.submit((position, timeout) -> StatusCode.OK, -5.0, 0.0));
  }

  @Test
  void preservesPhoenixFailureForDiagnostics() throws Exception {
    var future =
        StartupPositionWrite.submit((position, timeout) -> StatusCode.RxTimeout, -5.0, 0.1);
    assertEquals(StatusCode.RxTimeout, future.get(1, TimeUnit.SECONDS));
  }
}
