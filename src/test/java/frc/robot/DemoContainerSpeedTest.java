package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class DemoContainerSpeedTest {
  @Test
  void usesDefaultForInvalidDashboardSpeeds() {
    assertEquals(2.0, DemoContainer.sanitizeDashboardSpeed(Double.NaN, 2.0, 5.0));
    assertEquals(2.0, DemoContainer.sanitizeDashboardSpeed(0.0, 2.0, 5.0));
    assertEquals(2.0, DemoContainer.sanitizeDashboardSpeed(-1.0, 2.0, 5.0));
  }

  @Test
  void capsDefaultFallbackAtPhysicalMax() {
    assertEquals(1.0, DemoContainer.sanitizeDashboardSpeed(Double.NaN, 2.0, 1.0));
  }

  @Test
  void capsDashboardSpeedsAtPhysicalMax() {
    assertEquals(5.0, DemoContainer.sanitizeDashboardSpeed(8.0, 2.0, 5.0));
  }

  @Test
  void acceptsPositiveDashboardSpeedsUnderPhysicalMax() {
    assertEquals(1.5, DemoContainer.sanitizeDashboardSpeed(1.5, 2.0, 5.0));
  }
}
