package frc.robot.util;

public enum ShotTableTarget {
  HUB("shot_table_2d.json", "HUB"),
  FERRY("ferry_shot_table_2d.json", "FERRY");

  private final String deployFilename;
  private final String fileTargetName;

  ShotTableTarget(String deployFilename, String fileTargetName) {
    this.deployFilename = deployFilename;
    this.fileTargetName = fileTargetName;
  }

  public String deployFilename() {
    return deployFilename;
  }

  public String fileTargetName() {
    return fileTargetName;
  }

  public static ShotTableTarget fromCliName(String value) {
    for (ShotTableTarget target : values()) {
      if (target.name().equalsIgnoreCase(value)) {
        return target;
      }
    }
    throw new IllegalArgumentException("--target must be hub or ferry");
  }
}
