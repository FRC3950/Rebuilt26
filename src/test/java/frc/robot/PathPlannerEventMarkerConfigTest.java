package frc.robot;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.stream.Stream;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.junit.jupiter.api.Test;

class PathPlannerEventMarkerConfigTest {
  @Test
  void eventMarkersIncludeExecutableCommands() throws Exception {
    try (Stream<Path> pathFiles = Files.list(Path.of("src/main/deploy/pathplanner/paths"))) {
      for (Path pathFile : pathFiles.filter(path -> path.toString().endsWith(".path")).toList()) {
        JSONObject pathJson = parseJson(pathFile);
        JSONArray eventMarkers = (JSONArray) pathJson.get("eventMarkers");
        if (eventMarkers == null) {
          continue;
        }

        for (Object markerObject : eventMarkers) {
          JSONObject markerJson = (JSONObject) markerObject;
          assertNotNull(
              markerJson.get("command"),
              () -> "Event marker '" + markerJson.get("name") + "' in " + pathFile
                  + " is missing its command payload.");
        }
      }
    }
  }

  private static JSONObject parseJson(Path pathFile) throws Exception {
    try {
      return (JSONObject) new JSONParser().parse(Files.readString(pathFile));
    } catch (IOException exception) {
      throw new IOException("Failed to read " + pathFile, exception);
    }
  }
}
