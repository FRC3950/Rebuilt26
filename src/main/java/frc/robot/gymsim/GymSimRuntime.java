package frc.robot.gymsim;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.littletonrobotics.junction.Logger;

public final class GymSimRuntime {
  private static final Pattern STRING_FIELD =
      Pattern.compile("\"([^\"]+)\"\\s*:\\s*\"((?:\\\\.|[^\"])*)\"");

  private final Path runDir;
  private final Path requestDir;
  private final Path eventsFile;
  private final Object robotContainer;
  private final Map<String, Command> activeCommands = new HashMap<>();
  private final Map<Command, String> commandNames = new HashMap<>();
  private final Set<Path> processedRequests = new HashSet<>();
  private boolean active;
  private String lastEvent = "started";

  private GymSimRuntime(Path runDir, Object robotContainer) {
    this.runDir = runDir;
    this.requestDir = runDir.resolve("requests");
    this.eventsFile = runDir.resolve("events.jsonl");
    this.robotContainer = robotContainer;
    this.active = true;
    try {
      Files.createDirectories(requestDir);
      writeEvent("runtime_started", "", "ok");
    } catch (IOException ex) {
      DriverStation.reportError("GymSim failed to initialize: " + ex.getMessage(), false);
      this.active = false;
    }
  }

  public static GymSimRuntime start(Object robotContainer) {
    String runDir = System.getenv("GYMSIM_RUN_DIR");
    if (runDir == null || runDir.isBlank()) {
      return null;
    }
    return new GymSimRuntime(Path.of(runDir), robotContainer);
  }

  public void periodic() {
    if (!active) {
      return;
    }
    processRequests();
    processFinishedCommands();
    Logger.recordOutput("GymSim/Active", true);
    Logger.recordOutput("GymSim/RunDir", runDir.toString());
    Logger.recordOutput("GymSim/LastEvent", lastEvent);
    Logger.recordOutput("GymSim/ActiveCommandCount", activeCommands.size());
    Logger.recordOutput("GymSim/ProcessedRequestCount", processedRequests.size());
  }

  public void shutdown() {
    if (!active) {
      return;
    }
    CommandScheduler.getInstance().cancelAll();
    writeEvent("runtime_shutdown", "", "ok");
    Logger.recordOutput("GymSim/Active", false);
    Logger.end();
    active = false;
    new Thread(
            () -> {
              try {
                Thread.sleep(150);
              } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
              }
              System.exit(0);
            },
            "GymSimShutdown")
        .start();
  }

  private void processRequests() {
    try (var stream = Files.list(requestDir)) {
      stream
          .filter(path -> path.getFileName().toString().endsWith(".json"))
          .sorted()
          .forEach(this::processRequest);
    } catch (IOException ex) {
      writeEvent("request_scan_failed", "", ex.getMessage());
    }
  }

  private void processRequest(Path path) {
    if (!processedRequests.add(path)) {
      return;
    }
    try {
      String json = Files.readString(path, StandardCharsets.UTF_8);
      String id = field(json, "id").orElse(path.getFileName().toString());
      String type = field(json, "type").orElse("");
      switch (type) {
        case "driverstation" -> applyDriverStation(id, json);
        case "joystick" -> applyJoystick(id, json);
        case "schedule" -> scheduleCommand(id, field(json, "command").orElse(""));
        case "cancel" -> cancelCommand(field(json, "command").orElse(""));
        case "invoke" -> invokeExpression(id, field(json, "expression").orElse(""));
        case "marker" -> writeEvent("marker", field(json, "marker").orElse(id), "ok");
        case "shutdown" -> shutdown();
        default -> writeEvent("unknown_request", id, type);
      }
    } catch (IOException ex) {
      writeEvent("request_read_failed", path.getFileName().toString(), ex.getMessage());
    } catch (RuntimeException ex) {
      writeEvent("request_failed", path.getFileName().toString(), ex.getMessage());
    }
  }

  private void applyDriverStation(String requestId, String json) {
    String mode = field(json, "mode").orElse("teleop").toLowerCase();
    boolean enabled = Boolean.parseBoolean(field(json, "enabled").orElse("false"));
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(station(field(json, "station").orElse("blue1")));
    DriverStationSim.setAutonomous(mode.equals("autonomous"));
    DriverStationSim.setTest(mode.equals("test"));
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
    writeEvent("driverstation", mode, requestId + ":" + enabled);
  }

  private void applyJoystick(String requestId, String json) {
    int port = parseInt(field(json, "port").orElse("0"), 0);
    JoystickSim joystick = new JoystickSim(port);
    field(json, "axis")
        .ifPresent(
            axis -> {
              int index = parseInt(axis, -1);
              if (index >= 0) {
                joystick.setAxisCount(Math.max(index + 1, 6));
                joystick.setRawAxis(index, parseDouble(field(json, "value").orElse("0"), 0.0));
              }
            });
    field(json, "button")
        .ifPresent(
            button -> {
              int index = parseInt(button, -1);
              if (index > 0) {
                joystick.setButtonCount(Math.max(index, 12));
                joystick.setRawButton(
                    index, Boolean.parseBoolean(field(json, "pressed").orElse("false")));
              }
            });
    field(json, "pov")
        .ifPresent(
            pov -> {
              int index = parseInt(pov, -1);
              if (index >= 0) {
                joystick.setPOVCount(Math.max(index + 1, 1));
                joystick.setPOV(index, parseInt(field(json, "angle").orElse("-1"), -1));
              }
            });
    joystick.notifyNewData();
    writeEvent("joystick", Integer.toString(port), requestId);
  }

  private AllianceStationID station(String value) {
    return switch (value.toLowerCase()) {
      case "red1", "red" -> AllianceStationID.Red1;
      case "red2" -> AllianceStationID.Red2;
      case "red3" -> AllianceStationID.Red3;
      case "blue2" -> AllianceStationID.Blue2;
      case "blue3" -> AllianceStationID.Blue3;
      case "blue1", "blue" -> AllianceStationID.Blue1;
      default -> AllianceStationID.Unknown;
    };
  }

  private void scheduleCommand(String requestId, String expression) {
    if (expression.isBlank()) {
      writeEvent("expression_failed", expression, requestId + ":missing command expression");
      return;
    }
    Object result;
    try {
      result = evaluateExpression(expression);
    } catch (ExpressionException ex) {
      writeEvent("expression_failed", expression, requestId + ":" + ex.getMessage());
      return;
    }
    if (result == null) {
      writeEvent("command_null", expression, requestId);
      return;
    }
    if (!(result instanceof Command command)) {
      writeEvent("command_not_command", expression, requestId + ":" + result.getClass().getName());
      return;
    }
    activeCommands.put(expression, command);
    commandNames.put(command, expression);
    CommandScheduler.getInstance().schedule(command);
    writeEvent("command_scheduled", expression, requestId);
  }

  private void invokeExpression(String requestId, String expression) {
    if (expression.isBlank()) {
      writeEvent("expression_failed", expression, requestId + ":missing expression");
      return;
    }
    try {
      evaluateExpression(expression);
      writeEvent("expression_invoked", expression, requestId);
    } catch (ExpressionException ex) {
      writeEvent("expression_failed", expression, requestId + ":" + ex.getMessage());
    }
  }

  private void cancelCommand(String expression) {
    if (expression.isBlank()) {
      CommandScheduler.getInstance().cancelAll();
      activeCommands.clear();
      commandNames.clear();
      writeEvent("commands_cancelled", "", "all");
      return;
    }
    Command command = activeCommands.remove(expression);
    if (command == null) {
      writeEvent("command_cancel_missing", expression, "");
      return;
    }
    commandNames.remove(command);
    command.cancel();
    writeEvent("command_cancelled", expression, "");
  }

  private void processFinishedCommands() {
    var iterator = activeCommands.entrySet().iterator();
    while (iterator.hasNext()) {
      Map.Entry<String, Command> entry = iterator.next();
      if (!CommandScheduler.getInstance().isScheduled(entry.getValue())) {
        commandNames.remove(entry.getValue());
        writeEvent("command_finished", entry.getKey(), "");
        iterator.remove();
      }
    }
  }

  private Object evaluateExpression(String expression) throws ExpressionException {
    validateExpression(expression);
    Object current = robotContainer;
    if (current == null) {
      throw new ExpressionException("robotContainer is null");
    }
    List<String> segments = splitSegments(expression);
    for (int i = 0; i < segments.size(); i++) {
      String segment = segments.get(i);
      if (segment.endsWith(")")) {
        current = invokeMethod(current, segment);
      } else {
        current = readField(current, segment);
      }
      if (current == null && i < segments.size() - 1) {
        throw new ExpressionException("segment returned null: " + segment);
      }
    }
    return current;
  }

  private void validateExpression(String expression) throws ExpressionException {
    if (expression.contains(";")
        || expression.contains("=")
        || expression.contains("{")
        || expression.contains("}")
        || expression.contains("->")
        || expression.contains(" new ")) {
      throw new ExpressionException("unsupported expression syntax");
    }
  }

  private List<String> splitSegments(String expression) throws ExpressionException {
    List<String> segments = new ArrayList<>();
    int depth = 0;
    boolean inString = false;
    boolean escaped = false;
    int start = 0;
    for (int i = 0; i < expression.length(); i++) {
      char ch = expression.charAt(i);
      if (escaped) {
        escaped = false;
        continue;
      }
      if (ch == '\\' && inString) {
        escaped = true;
        continue;
      }
      if (ch == '"') {
        inString = !inString;
        continue;
      }
      if (inString) {
        continue;
      }
      if (ch == '(') {
        depth++;
      } else if (ch == ')') {
        depth--;
        if (depth < 0) {
          throw new ExpressionException("unbalanced parentheses");
        }
      } else if (ch == '.' && depth == 0) {
        addSegment(segments, expression.substring(start, i));
        start = i + 1;
      }
    }
    if (inString || depth != 0) {
      throw new ExpressionException("unbalanced expression");
    }
    addSegment(segments, expression.substring(start));
    return segments;
  }

  private void addSegment(List<String> segments, String segment) throws ExpressionException {
    String trimmed = segment.trim();
    if (trimmed.isBlank()) {
      throw new ExpressionException("empty expression segment");
    }
    segments.add(trimmed);
  }

  private Object readField(Object target, String fieldName) throws ExpressionException {
    if (!isIdentifier(fieldName)) {
      throw new ExpressionException("invalid field name: " + fieldName);
    }
    Field field = findField(target.getClass(), fieldName);
    if (field == null) {
      throw new ExpressionException("field not found: " + fieldName);
    }
    try {
      field.setAccessible(true);
      return field.get(target);
    } catch (IllegalAccessException ex) {
      throw new ExpressionException("field access failed: " + fieldName);
    }
  }

  private Field findField(Class<?> type, String name) {
    Class<?> current = type;
    while (current != null) {
      try {
        return current.getDeclaredField(name);
      } catch (NoSuchFieldException ex) {
        current = current.getSuperclass();
      }
    }
    return null;
  }

  private Object invokeMethod(Object target, String segment) throws ExpressionException {
    int open = segment.indexOf('(');
    if (open <= 0 || !segment.endsWith(")")) {
      throw new ExpressionException("invalid method call: " + segment);
    }
    String methodName = segment.substring(0, open).trim();
    if (!isIdentifier(methodName)) {
      throw new ExpressionException("invalid method name: " + methodName);
    }
    List<String> args = splitArguments(segment.substring(open + 1, segment.length() - 1));
    MethodMatch match = findMethod(target.getClass(), methodName, args);
    try {
      match.method().setAccessible(true);
      return match.method().invoke(target, match.arguments());
    } catch (IllegalAccessException ex) {
      throw new ExpressionException("method access failed: " + methodName);
    } catch (InvocationTargetException ex) {
      Throwable cause = ex.getCause();
      String message = cause == null ? ex.getMessage() : cause.getMessage();
      throw new ExpressionException("method threw: " + methodName + ": " + message);
    }
  }

  private MethodMatch findMethod(Class<?> type, String name, List<String> args)
      throws ExpressionException {
    List<MethodMatch> matches = new ArrayList<>();
    Class<?> current = type;
    while (current != null) {
      for (Method method : current.getDeclaredMethods()) {
        if (!method.getName().equals(name) || method.getParameterCount() != args.size()) {
          continue;
        }
        Optional<Object[]> converted = convertArguments(method.getParameterTypes(), args);
        converted.ifPresent(values -> matches.add(new MethodMatch(method, values)));
      }
      current = current.getSuperclass();
    }
    if (matches.isEmpty()) {
      throw new ExpressionException("method not found: " + name);
    }
    if (matches.size() > 1) {
      throw new ExpressionException("ambiguous method: " + name);
    }
    return matches.get(0);
  }

  private Optional<Object[]> convertArguments(Class<?>[] parameterTypes, List<String> args) {
    Object[] converted = new Object[args.size()];
    for (int i = 0; i < args.size(); i++) {
      Optional<Object> value = convertArgument(parameterTypes[i], args.get(i));
      if (value.isEmpty()) {
        return Optional.empty();
      }
      converted[i] = value.get();
    }
    return Optional.of(converted);
  }

  @SuppressWarnings({"unchecked", "rawtypes"})
  private Optional<Object> convertArgument(Class<?> type, String raw) {
    String value = raw.trim();
    if (type == String.class) {
      return Optional.of(
          isQuoted(value) ? unescapeString(value.substring(1, value.length() - 1)) : value);
    }
    if (type == boolean.class || type == Boolean.class) {
      if (value.equalsIgnoreCase("true") || value.equalsIgnoreCase("false")) {
        return Optional.of(Boolean.parseBoolean(value));
      }
      return Optional.empty();
    }
    if (type == int.class || type == Integer.class) {
      try {
        return Optional.of(Integer.parseInt(value));
      } catch (NumberFormatException ex) {
        return Optional.empty();
      }
    }
    if (type == double.class || type == Double.class) {
      try {
        return Optional.of(Double.parseDouble(value));
      } catch (NumberFormatException ex) {
        return Optional.empty();
      }
    }
    if (type.isEnum()) {
      String enumValue =
          isQuoted(value) ? unescapeString(value.substring(1, value.length() - 1)) : value;
      for (Object constant : type.getEnumConstants()) {
        if (((Enum) constant).name().equals(enumValue)) {
          return Optional.of(constant);
        }
      }
    }
    return Optional.empty();
  }

  private List<String> splitArguments(String text) throws ExpressionException {
    List<String> args = new ArrayList<>();
    String trimmed = text.trim();
    if (trimmed.isBlank()) {
      return args;
    }
    boolean inString = false;
    boolean escaped = false;
    int start = 0;
    for (int i = 0; i < text.length(); i++) {
      char ch = text.charAt(i);
      if (escaped) {
        escaped = false;
        continue;
      }
      if (ch == '\\' && inString) {
        escaped = true;
        continue;
      }
      if (ch == '"') {
        inString = !inString;
      } else if (ch == ',' && !inString) {
        addArgument(args, text.substring(start, i));
        start = i + 1;
      }
    }
    if (inString) {
      throw new ExpressionException("unterminated string argument");
    }
    addArgument(args, text.substring(start));
    return args;
  }

  private void addArgument(List<String> args, String arg) throws ExpressionException {
    String trimmed = arg.trim();
    if (trimmed.isBlank()) {
      throw new ExpressionException("empty argument");
    }
    args.add(trimmed);
  }

  private boolean isIdentifier(String value) {
    return value.matches("[A-Za-z_$][A-Za-z0-9_$]*");
  }

  private boolean isQuoted(String value) {
    return value.length() >= 2 && value.startsWith("\"") && value.endsWith("\"");
  }

  private String unescapeString(String value) {
    return value.replace("\\\"", "\"").replace("\\\\", "\\");
  }

  private Optional<String> field(String json, String name) {
    Matcher matcher = STRING_FIELD.matcher(json);
    while (matcher.find()) {
      if (matcher.group(1).equals(name)) {
        return Optional.of(unescape(matcher.group(2)));
      }
    }
    return Optional.empty();
  }

  private int parseInt(String value, int fallback) {
    try {
      return Integer.parseInt(value);
    } catch (NumberFormatException ex) {
      return fallback;
    }
  }

  private double parseDouble(String value, double fallback) {
    try {
      return Double.parseDouble(value);
    } catch (NumberFormatException ex) {
      return fallback;
    }
  }

  private String unescape(String value) {
    return value.replace("\\\"", "\"").replace("\\\\", "\\");
  }

  private void writeEvent(String event, String command, String detail) {
    lastEvent = event;
    String json =
        "{\"time\":\""
            + escape(Instant.now().toString())
            + "\",\"event\":\""
            + escape(event)
            + "\",\"command\":\""
            + escape(command)
            + "\",\"detail\":\""
            + escape(detail)
            + "\"}\n";
    try {
      Files.writeString(
          eventsFile,
          json,
          StandardCharsets.UTF_8,
          java.nio.file.StandardOpenOption.CREATE,
          java.nio.file.StandardOpenOption.APPEND);
    } catch (IOException ex) {
      DriverStation.reportError("GymSim failed to write event: " + ex.getMessage(), false);
    }
  }

  private String escape(String value) {
    return value.replace("\\", "\\\\").replace("\"", "\\\"");
  }

  private record MethodMatch(Method method, Object[] arguments) {}

  private static final class ExpressionException extends Exception {
    private ExpressionException(String message) {
      super(message);
    }
  }
}
