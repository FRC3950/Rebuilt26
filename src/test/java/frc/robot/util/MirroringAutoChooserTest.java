package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class MirroringAutoChooserTest {
  @Test
  void buildsNoneWhenNoAutoIsSelected() {
    AtomicBoolean factoryCalled = new AtomicBoolean(false);

    Command command =
        MirroringAutoChooser.buildAutonomousCommand(
            "",
            false,
            (autoName, mirror) -> {
              factoryCalled.set(true);
              throw new AssertionError("No auto should be built for an empty selection.");
            });

    assertFalse(factoryCalled.get());
    assertNotNull(command);
  }

  @Test
  void buildsSelectedAutoWithoutMirrorWhenSwitchIsFalse() {
    Command expectedCommand = Commands.none();
    AtomicReference<String> builtAutoName = new AtomicReference<>();
    AtomicBoolean builtMirror = new AtomicBoolean(true);

    Command command =
        MirroringAutoChooser.buildAutonomousCommand(
            "SweepyFerry",
            false,
            (autoName, mirror) -> {
              builtAutoName.set(autoName);
              builtMirror.set(mirror);
              return expectedCommand;
            });

    assertSame(expectedCommand, command);
    assertEquals("SweepyFerry", builtAutoName.get());
    assertFalse(builtMirror.get());
  }

  @Test
  void buildsSelectedAutoWithMirrorWhenSwitchIsTrue() {
    Command expectedCommand = Commands.none();
    AtomicReference<String> builtAutoName = new AtomicReference<>();
    AtomicBoolean builtMirror = new AtomicBoolean(false);

    Command command =
        MirroringAutoChooser.buildAutonomousCommand(
            "SweepyFerry",
            true,
            (autoName, mirror) -> {
              builtAutoName.set(autoName);
              builtMirror.set(mirror);
              return expectedCommand;
            });

    assertSame(expectedCommand, command);
    assertEquals("SweepyFerry", builtAutoName.get());
    assertTrue(builtMirror.get());
  }
}
