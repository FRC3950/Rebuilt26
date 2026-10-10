package frc.robot.subsystems.turret.turret_base;

/** Allows one startup recovery attempt at enable, never after a failure during an enabled run. */
final class StartupEnableRecovery {
  private boolean wasEnabled;

  boolean shouldAttempt(boolean enabled, boolean ready, boolean initializing, boolean allowed) {
    boolean justEnabled = enabled && !wasEnabled;
    wasEnabled = enabled;
    return justEnabled && !ready && !initializing && allowed;
  }
}
