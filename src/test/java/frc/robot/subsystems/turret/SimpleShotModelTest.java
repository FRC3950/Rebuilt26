package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class SimpleShotModelTest {
  private final Translation2d target = new Translation2d(4.6256194, 4.0346376);
  private final Translation2d left = Constants.SubsystemConstants.Turret.robotToTurret1;
  private final Translation2d right = Constants.SubsystemConstants.Turret.robotToTurret2;

  private void verifyVacuum(
      SimpleShotModel.Settings settings,
      Pose2d pose,
      ChassisSpeeds motion,
      Translation2d offset,
      double roll,
      double pitch) {
    var s = ReferenceShotPhysics.solve(settings, pose, motion, target, offset, roll, pitch);
    assertTrue(s.parameters().isValid(), s.parameters().invalidReason());
    var p = s.parameters();
    double t = s.timeOfFlight(), delay = settings.get(SimpleShotModel.Param.DELAY);
    double heading = pose.getRotation().getRadians() + motion.omegaRadiansPerSecond * delay;
    var o = offset.rotateBy(Rotation2d.fromRadians(heading));
    double hood = p.hoodAngleDeg(),
        e =
            settings.get(SimpleShotModel.Param.ELEV_MIN)
                + (hood - 13)
                    * (settings.get(SimpleShotModel.Param.ELEV_MAX)
                        - settings.get(SimpleShotModel.Param.ELEV_MIN))
                    / (29.85 - 13);
    double speed =
        p.flywheelSpeed()
            * 2
            * Math.PI
            * settings.get(SimpleShotModel.Param.RADIUS)
            * settings.get(SimpleShotModel.Param.TRANSFER);
    double a = p.turretAngle().getRadians(), h = speed * Math.cos(Math.toRadians(e));
    double x = h * Math.cos(a), y = h * Math.sin(a), z = speed * Math.sin(Math.toRadians(e));
    double ry = Math.cos(roll) * y - Math.sin(roll) * z,
        rz = Math.sin(roll) * y + Math.cos(roll) * z;
    double rx = Math.cos(pitch) * x + Math.sin(pitch) * rz,
        zz = -Math.sin(pitch) * x + Math.cos(pitch) * rz;
    double wx = Math.cos(heading) * rx - Math.sin(heading) * ry,
        wy = Math.sin(heading) * rx + Math.cos(heading) * ry;
    double vx = motion.vxMetersPerSecond - motion.omegaRadiansPerSecond * o.getY();
    double vy = motion.vyMetersPerSecond + motion.omegaRadiansPerSecond * o.getX();
    assertEquals(
        target.getX(),
        pose.getX() + motion.vxMetersPerSecond * delay + o.getX() + (wx + vx) * t,
        1e-6);
    assertEquals(
        target.getY(),
        pose.getY() + motion.vyMetersPerSecond * delay + o.getY() + (wy + vy) * t,
        1e-6);
    assertEquals(
        SimpleShotModel.HUB_HEIGHT,
        settings.get(SimpleShotModel.Param.HEIGHT) + zz * t - .5 * 9.81 * t * t,
        1e-6);
    assertTrue(zz - 9.81 * t < 0, "Hub entry must be descending");
  }

  @Test
  void independentReferenceReconstructsBothTurretsAndMovingTiltedRobot() {
    var c = SimpleShotModel.defaults();
    for (var offset : new Translation2d[] {left, right}) {
      verifyVacuum(c, new Pose2d(1, 3, Rotation2d.kZero), new ChassisSpeeds(), offset, 0, 0);
      verifyVacuum(
          c,
          new Pose2d(1, 3, Rotation2d.fromDegrees(40)),
          new ChassisSpeeds(1.2, -1.8, .8),
          offset,
          .03,
          -.04);
      verifyVacuum(
          c.with(SimpleShotModel.Param.DELAY, .1),
          new Pose2d(1, 3, Rotation2d.fromDegrees(40)),
          new ChassisSpeeds(1.2, -1.8, .8),
          offset,
          .03,
          -.04);
    }
  }

  @Test
  void closeStationaryShotsMustEnterWhileDescending() {
    verifyVacuum(
        SimpleShotModel.defaults(),
        new Pose2d(3.9, 4, Rotation2d.kZero),
        new ChassisSpeeds(),
        new Translation2d(),
        0,
        0);
  }

  @Test
  void transferAndSideMultipliersHavePredictableDirections() {
    var c = SimpleShotModel.defaults();
    double rps = SimpleShotModel.wheelRps(c, 8, true);
    assertEquals(
        2 * rps,
        SimpleShotModel.wheelRps(c.with(SimpleShotModel.Param.TRANSFER, .5), 8, true),
        1e-10);
    assertEquals(
        rps * 1.1,
        SimpleShotModel.wheelRps(c.with(SimpleShotModel.Param.LEFT_SPEED, 1.1), 8, true),
        1e-10);
    assertEquals(
        rps,
        SimpleShotModel.wheelRps(c.with(SimpleShotModel.Param.LEFT_SPEED, 1.1), 8, false),
        1e-10);
  }

  @Test
  void optionalDragAndBackspinCorrectionsAreIndependentAndBounded() {
    var c = SimpleShotModel.defaults();
    var pose = new Pose2d(1, 3, Rotation2d.kZero);
    var base = ReferenceShotPhysics.solve(c, pose, new ChassisSpeeds(), target, left, 0, 0);
    var drag =
        ReferenceShotPhysics.solve(
            c.with(SimpleShotModel.Param.DRAG, .3), pose, new ChassisSpeeds(), target, left, 0, 0);
    var lift =
        ReferenceShotPhysics.solve(
            c.with(SimpleShotModel.Param.DRAG, .3)
                .with(SimpleShotModel.Param.TRANSFER, .5)
                .with(SimpleShotModel.Param.SPIN_RATIO, .3)
                .with(SimpleShotModel.Param.LIFT, .1),
            pose,
            new ChassisSpeeds(),
            target,
            left,
            0,
            0);
    assertTrue(drag.parameters().isValid(), drag.parameters().invalidReason());
    assertTrue(lift.parameters().isValid(), lift.parameters().invalidReason());
    assertTrue(drag.parameters().flywheelSpeed() > base.parameters().flywheelSpeed());
    // Magnus lift also changes horizontal acceleration; fixed-angle speed need not decrease.
    assertNotEquals(2 * drag.parameters().flywheelSpeed(), lift.parameters().flywheelSpeed(), 1e-4);
    assertEquals(0, c.get(SimpleShotModel.Param.DRAG));
    assertEquals(0, c.get(SimpleShotModel.Param.LIFT));
  }

  @Test
  void rejectsInvalidSettingsAndRealLimitsInsteadOfClampingWrongShot() {
    var c = SimpleShotModel.defaults();
    assertFalse(c.with(SimpleShotModel.Param.TRANSFER, 0).error().isEmpty());
    assertFalse(
        ReferenceShotPhysics.solve(
                c.with(SimpleShotModel.Param.TRANSFER, .01),
                new Pose2d(1, 3, Rotation2d.kZero),
                new ChassisSpeeds(),
                target,
                left,
                0,
                0)
            .parameters()
            .isValid());
    assertFalse(
        ReferenceShotPhysics.solve(
                c, new Pose2d(), new ChassisSpeeds(), target, left, Double.NaN, 0)
            .parameters()
            .isValid());
  }

  @Test
  void dashboardSettingsAreLiveAndSaveResetAreExplicitOneShotActions() {
    assertTrue(HAL.initialize(500, 0));
    try (var instance = NetworkTableInstance.create()) {
      instance.startLocal();
      SmartDashboard.setNetworkTableInstance(instance);
      Preferences.setNetworkTableInstance(instance);
      try {
        SimpleShotModel.initializeDashboard();
        String transfer = SimpleShotModel.Param.TRANSFER.key;
        SmartDashboard.putNumber(transfer, .6);
        SimpleShotModel.refreshDashboard();
        assertFalse(Preferences.containsKey(transfer));
        SmartDashboard.putBoolean("ShotModel/Save Settings", true);
        SimpleShotModel.refreshDashboard();
        assertEquals(.6, Preferences.getDouble(transfer, -1));
        assertFalse(SmartDashboard.getBoolean("ShotModel/Save Settings", true));
        SmartDashboard.putNumber(transfer, .7);
        SimpleShotModel.initializeDashboard();
        assertEquals(.6, SmartDashboard.getNumber(transfer, -1));
        SmartDashboard.putBoolean("ShotModel/Reset To Baseline", true);
        SimpleShotModel.refreshDashboard();
        assertEquals(1, SmartDashboard.getNumber(transfer, -1));
        assertEquals(.6, Preferences.getDouble(transfer, -1));
      } finally {
        SmartDashboard.setNetworkTableInstance(NetworkTableInstance.getDefault());
        Preferences.setNetworkTableInstance(NetworkTableInstance.getDefault());
        SimpleShotModel.initializeDashboard();
      }
    }
  }

  @Test
  void allianceRotationPreservesRobotRelativeOutputs() {
    var c = SimpleShotModel.defaults();
    var blue =
        ReferenceShotPhysics.solve(
            c,
            new Pose2d(1, 3, Rotation2d.kZero),
            new ChassisSpeeds(1, -.5, .4),
            target,
            left,
            0,
            0);
    var red =
        ReferenceShotPhysics.solve(
            c,
            new Pose2d(16.5411785 - 1, 8.0756125 - 3, Rotation2d.fromRadians(Math.PI)),
            new ChassisSpeeds(-1, .5, .4),
            new Translation2d(16.5411785 - target.getX(), 8.0756125 - target.getY()),
            left,
            0,
            0);
    assertTrue(blue.parameters().isValid());
    assertTrue(red.parameters().isValid());
    assertEquals(blue.parameters().hoodAngleDeg(), red.parameters().hoodAngleDeg(), 1e-8);
    assertEquals(blue.parameters().flywheelSpeed(), red.parameters().flywheelSpeed(), 1e-8);
    assertEquals(
        0,
        blue.parameters().turretAngle().minus(red.parameters().turretAngle()).getRadians(),
        1e-8);
  }

  @Test
  void stationaryAllianceZoneUsesModelAcrossBothTurretOffsets() {
    var c = SimpleShotModel.defaults();
    for (int ix = 0; ix <= 10; ix++)
      for (int iy = 0; iy <= 16; iy++) {
        double x = ix * 5.1816 / 10, y = iy * 8.0756125 / 16;
        if (Math.abs(x - target.getX()) < 1.05 && Math.abs(y - target.getY()) < 1.05) continue;
        for (var offset : new Translation2d[] {left, right})
          for (double heading : new double[] {0, Math.PI / 2, Math.PI, 3 * Math.PI / 2})
            verifyVacuum(
                c,
                new Pose2d(x, y, Rotation2d.fromRadians(heading)),
                new ChassisSpeeds(),
                offset,
                0,
                0);
      }
  }

  @Test
  void exportsReferenceCasesForIndependentFlightCheck() throws Exception {
    var output = new java.util.ArrayList<java.util.Map<String, Object>>();
    for (double drag : new double[] {0, .3})
      for (double lift : new double[] {0, .05})
        for (double lateral : new double[] {0, 1.5}) {
          var c =
              SimpleShotModel.defaults()
                  .with(SimpleShotModel.Param.TRANSFER, .5)
                  .with(SimpleShotModel.Param.DRAG, drag)
                  .with(SimpleShotModel.Param.SPIN_RATIO, .3)
                  .with(SimpleShotModel.Param.LIFT, lift);
          var pose = new Pose2d(1, 3, Rotation2d.kZero);
          var s =
              ReferenceShotPhysics.solve(
                  c, pose, new ChassisSpeeds(.5, lateral, .2), target, left, 0, 0);
          assertTrue(s.parameters().isValid(), s.parameters().invalidReason());
          double angle =
              c.get(SimpleShotModel.Param.ELEV_MIN)
                  + (s.parameters().hoodAngleDeg() - 13)
                      * (c.get(SimpleShotModel.Param.ELEV_MAX)
                          - c.get(SimpleShotModel.Param.ELEV_MIN))
                      / (29.85 - 13);
          double bearing =
              Math.atan2(
                  target.getY() - pose.getY() - left.getY(),
                  target.getX() - pose.getX() - left.getX());
          output.add(
              java.util.Map.of(
                  "distance",
                  s.distance(),
                  "radial",
                  s.radial(),
                  "tangential",
                  s.tangential(),
                  "elevation",
                  angle,
                  "speed",
                  s.parameters().flywheelSpeed()
                      * 2
                      * Math.PI
                      * c.get(SimpleShotModel.Param.RADIUS)
                      * c.get(SimpleShotModel.Param.TRANSFER),
                  "spin",
                  s.parameters().flywheelSpeed() * .3,
                  "yaw",
                  s.parameters().turretAngle().getRadians() - bearing,
                  "drag",
                  drag,
                  "lift",
                  lift,
                  "time",
                  s.timeOfFlight()));
        }
    var directory = java.nio.file.Path.of("build/shot-lab");
    java.nio.file.Files.createDirectories(directory);
    new com.fasterxml.jackson.databind.ObjectMapper()
        .writeValue(directory.resolve("simple-reference.json").toFile(), output);
    var c = SimpleShotModel.defaults().with(SimpleShotModel.Param.DRAG, .3);
    var pose = new Pose2d(1, 3, Rotation2d.kZero);
    var motion = new ChassisSpeeds(1, 1, .3);
    for (int i = 0; i < 30; i++) ReferenceShotPhysics.solve(c, pose, motion, target, left, 0, 0);
    long start = System.nanoTime();
    for (int i = 0; i < 1000; i++)
      assertTrue(
          ReferenceShotPhysics.solve(c, pose, motion, target, left, 0, 0).parameters().isValid());
    System.out.printf(
        "Desktop optional-drag solve average: %.3f ms%n", (System.nanoTime() - start) / 1e6 / 1000);
  }

  @Test
  void ferryStillUsesOriginalTablePath() {
    var c = new GetAdjustedShot();
    assertTrue(
        c.getParameters(
                new Pose2d(1, 3, Rotation2d.kZero),
                new ChassisSpeeds(),
                new Translation2d(4, 3),
                new Translation2d())
            .isValid());
  }
}
