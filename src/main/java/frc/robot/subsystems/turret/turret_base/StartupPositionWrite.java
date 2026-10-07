package frc.robot.subsystems.turret.turret_base;

import com.ctre.phoenix6.StatusCode;
import java.util.concurrent.CompletableFuture;
import java.util.function.BiFunction;

/** Runs the bounded Phoenix position write without blocking the command scheduler. */
public final class StartupPositionWrite {
  private StartupPositionWrite() {}

  public static CompletableFuture<StatusCode> submit(
      BiFunction<Double, Double, StatusCode> writer, double positionRotations, double timeoutSec) {
    if (!Double.isFinite(positionRotations) || !Double.isFinite(timeoutSec) || timeoutSec <= 0.0) {
      throw new IllegalArgumentException(
          "Startup position write needs a finite position and positive timeout");
    }
    return CompletableFuture.supplyAsync(() -> writer.apply(positionRotations, timeoutSec));
  }
}
