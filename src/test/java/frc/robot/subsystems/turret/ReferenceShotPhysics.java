package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.SimpleShotModel.hoodForElevation;
import static frc.robot.subsystems.turret.SimpleShotModel.wheelRps;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.SubsystemConstants.Turret;
import frc.robot.subsystems.turret.SimpleShotModel.Param;
import frc.robot.subsystems.turret.SimpleShotModel.Settings;
import frc.robot.subsystems.turret.SimpleShotModel.Solution;

/**
 * Independent trajectory-search reference used only by desktop physics tests. Never ships on the
 * robot.
 */
final class ReferenceShotPhysics {
  private static final double G = 9.81,
      HUB_HEIGHT = 1.83,
      BALL_RADIUS = .075,
      HUB_RADIUS = .56,
      BALL_MASS = .448 * .45392,
      AIR_DENSITY = 1.2041,
      STEP = .01;

  private record Flight(double x, double y, double time) {}

  private record Aim(double speed, double yaw, double time) {}
  /** Reference solver for physics cross-checks only; never called by targeting. */
  public static Solution solve(
      Settings c,
      Pose2d pose,
      ChassisSpeeds velocity,
      Translation2d target,
      Translation2d offset,
      double roll,
      double pitch) {
    double distance = 0, radial = 0, tangential = 0;
    try {
      check(c.error().isEmpty(), c.error());
      check(
          finite(
              pose.getX(),
              pose.getY(),
              pose.getRotation().getRadians(),
              velocity.vxMetersPerSecond,
              velocity.vyMetersPerSecond,
              velocity.omegaRadiansPerSecond,
              target.getX(),
              target.getY(),
              roll,
              pitch),
          "Non-finite robot state / tilt unavailable");
      double delay = c.get(Param.DELAY),
          heading = pose.getRotation().getRadians() + velocity.omegaRadiansPerSecond * delay;
      Translation2d fieldOffset = offset.rotateBy(Rotation2d.fromRadians(heading));
      double dx =
          target.getX() - pose.getX() - velocity.vxMetersPerSecond * delay - fieldOffset.getX();
      double dy =
          target.getY() - pose.getY() - velocity.vyMetersPerSecond * delay - fieldOffset.getY();
      distance = Math.hypot(dx, dy);
      check(distance > 1e-6, "Turret coincides with target");
      double bearing = Math.atan2(dy, dx);
      double vx = velocity.vxMetersPerSecond - velocity.omegaRadiansPerSecond * fieldOffset.getY();
      double vy = velocity.vyMetersPerSecond + velocity.omegaRadiansPerSecond * fieldOffset.getX();
      radial = (vx * dx + vy * dy) / distance;
      tangential = (-vx * dy + vy * dx) / distance;
      double low = Math.min(c.get(Param.ELEV_MIN), c.get(Param.ELEV_MAX));
      double high = Math.max(c.get(Param.ELEV_MIN), c.get(Param.ELEV_MAX));
      double preferred = Math.max(low, Math.min(high, (low + high) / 2));
      String failure = "No descending shot with current angle mapping and motion";
      for (int attempt = 0; attempt < 9; attempt++) {
        double elevation =
            attempt <= 4
                ? preferred + (high - preferred) * attempt / 4.0
                : preferred - (preferred - low) * (attempt - 4) / 4.0;
        try {
          Aim aim =
              idealAim(
                  distance, radial, tangential, c.get(Param.HEIGHT), Math.toRadians(elevation));
          boolean left = offset.getY() >= 0;
          if (c.get(Param.DRAG) != 0 || (c.get(Param.LIFT) != 0 && c.get(Param.SPIN_RATIO) != 0)) {
            aim = correctedAim(c, distance, radial, tangential, elevation, aim, left);
          }
          // Rotate the desired world launch vector into the tilted robot frame.
          double localYaw = bearing + aim.yaw - heading;
          double horizontal = aim.speed * Math.cos(Math.toRadians(elevation));
          double x = horizontal * Math.cos(localYaw),
              y = horizontal * Math.sin(localYaw),
              z = aim.speed * Math.sin(Math.toRadians(elevation));
          double tx = Math.cos(pitch) * x - Math.sin(pitch) * z;
          double tz = Math.sin(pitch) * x + Math.cos(pitch) * z;
          double ty = Math.cos(roll) * y + Math.sin(roll) * tz;
          double zz = -Math.sin(roll) * y + Math.cos(roll) * tz;
          double physicalAngle = Math.toDegrees(Math.atan2(zz, Math.hypot(tx, ty)));
          double hood =
              hoodForElevation(c, physicalAngle) + c.get(left ? Param.LEFT_HOOD : Param.RIGHT_HOOD);
          double rps = wheelRps(c, aim.speed, left);
          check(
              finite(hood, rps, aim.time)
                  && hood >= Turret.minHoodAngle - 1e-6
                  && hood <= Turret.maxHoodAngle + 1e-6,
              "Hood limit: check angle mapping or trim");
          check(
              rps >= 0 && rps <= Turret.maxFlywheelRps,
              "Wheel RPS limit: check transfer, radius or corrections");
          double yaw =
              Math.atan2(ty, tx) + Math.toRadians(c.get(left ? Param.LEFT_YAW : Param.RIGHT_YAW));
          return new Solution(
              new GetAdjustedShot.ShootingParameters(
                  true,
                  Rotation2d.fromRadians(yaw),
                  0,
                  Math.max(Turret.minHoodAngle, Math.min(Turret.maxHoodAngle, hood)),
                  rps,
                  ""),
              distance,
              radial,
              tangential,
              aim.time);
        } catch (IllegalArgumentException e) {
          failure = e.getMessage();
        }
      }
      throw new IllegalArgumentException(failure);
    } catch (IllegalArgumentException e) {
      return new Solution(
          new GetAdjustedShot.ShootingParameters(false, Rotation2d.kZero, 0, 0, 0, e.getMessage()),
          distance,
          radial,
          tangential,
          0);
    }
  }

  private static Aim idealAim(
      double d, double radial, double tangential, double height, double angle) {
    double tan = Math.tan(angle), dz = HUB_HEIGHT - height;
    double descending = Math.sqrt(2 * dz / G) + 1e-6;
    double v = Math.hypot(radial, tangential) * tan;
    double hi = (v + Math.sqrt(v * v + 2 * G * Math.max(0, d * tan - dz))) / G + .01;
    check(hi > descending, "No descending shot at this elevation");
    double upper = hi, lo = Double.NaN;
    // Search from the latest possible crossing so moving shots cannot select an ascending entry.
    for (int i = 1; i <= 96; i++) {
      double time = upper - (upper - descending) * i / 96;
      if (residual(d, radial, tangential, dz, tan, time) > 0) {
        lo = time;
        break;
      }
      hi = time;
    }
    check(
        Double.isFinite(lo),
        "No descending shot: check launch-angle mapping or preferred elevation");
    for (int i = 0; i < 40; i++) {
      double time = (lo + hi) / 2;
      if (residual(d, radial, tangential, dz, tan, time) > 0) lo = time;
      else hi = time;
    }
    double t = (lo + hi) / 2, x = d / t - radial, y = -tangential;
    check(t > 0 && finite(t, x, y), "Invalid ballistic solution");
    return new Aim(Math.hypot(x, y) / Math.cos(angle), Math.atan2(y, x), t);
  }

  private static double residual(
      double d, double radial, double tangential, double dz, double tan, double t) {
    return Math.hypot(d - radial * t, tangential * t) * tan - dz - .5 * G * t * t;
  }

  private static Aim correctedAim(
      Settings c,
      double d,
      double radial,
      double tangential,
      double elevation,
      Aim initial,
      boolean left) {
    double speed = initial.speed, yaw = initial.yaw;
    for (int i = 0; i < 8; i++) {
      Flight f = flight(c, speed, elevation, yaw, radial, tangential, left);
      check(f != null, "Correction trajectory did not reach hub height");
      double ex = f.x - d, ey = f.y;
      if (Math.hypot(ex, ey) < .005) return new Aim(speed, yaw, f.time);
      double ds = .02, da = .002;
      Flight vs = flight(c, speed + ds, elevation, yaw, radial, tangential, left),
          va = flight(c, speed, elevation, yaw + da, radial, tangential, left);
      check(vs != null && va != null, "Correction derivative unavailable");
      double a = (vs.x - f.x) / ds,
          b = (va.x - f.x) / da,
          cc = (vs.y - f.y) / ds,
          e = (va.y - f.y) / da,
          det = a * e - b * cc;
      check(Math.abs(det) > 1e-8, "Correction solver singular; reduce advanced corrections");
      speed -= Math.max(-2, Math.min(2, (e * ex - b * ey) / det));
      yaw -= Math.max(-.2, Math.min(.2, (-cc * ex + a * ey) / det));
      check(speed > 0 && speed < 60, "Correction speed diverged; recheck coefficients");
    }
    Flight f = flight(c, speed, elevation, yaw, radial, tangential, left);
    check(
        f != null && Math.hypot(f.x - d, f.y) < Math.min(.03, HUB_RADIUS - BALL_RADIUS),
        "Correction solver did not converge; reduce advanced coefficients");
    return new Aim(speed, yaw, f.time);
  }

  /** Bounded flight integration is only used when optional drag/lift corrections are nonzero. */
  private static Flight flight(
      Settings c,
      double speed,
      double elevation,
      double yaw,
      double radial,
      double tangential,
      boolean left) {
    double angle = Math.toRadians(elevation),
        h = speed * Math.cos(angle),
        vx = h * Math.cos(yaw) + radial,
        vy = h * Math.sin(yaw) + tangential,
        vz = speed * Math.sin(angle);
    double x = 0,
        y = 0,
        z = c.get(Param.HEIGHT),
        spin = wheelRps(c, speed, left) * c.get(Param.SPIN_RATIO) * 2 * Math.PI;
    double drag =
        .5 * AIR_DENSITY * Math.PI * BALL_RADIUS * BALL_RADIUS * c.get(Param.DRAG) / BALL_MASS;
    double lift =
        .5
            * AIR_DENSITY
            * Math.PI
            * BALL_RADIUS
            * BALL_RADIUS
            * c.get(Param.LIFT)
            * BALL_RADIUS
            / BALL_MASS;
    for (int i = 0; i < 500; i++) {
      double oldZ = z, oldX = x, oldY = y, t = i * STEP;
      double norm = Math.sqrt(vx * vx + vy * vy + vz * vz),
          omega = spin * Math.exp(-c.get(Param.SPIN_DECAY) * t);
      double sx = Math.sin(yaw) * omega, sy = -Math.cos(yaw) * omega;
      double ax = -drag * norm * vx + lift * norm * sy * vz;
      double ay = -drag * norm * vy - lift * norm * sx * vz;
      double az = -G - drag * norm * vz + lift * norm * (sx * vy - sy * vx);
      // Midpoint integration: accurate gravity/drag without per-step objects or arrays.
      double mx = vx + ax * STEP / 2, my = vy + ay * STEP / 2, mz = vz + az * STEP / 2;
      norm = Math.sqrt(mx * mx + my * my + mz * mz);
      omega = spin * Math.exp(-c.get(Param.SPIN_DECAY) * (t + STEP / 2));
      sx = Math.sin(yaw) * omega;
      sy = -Math.cos(yaw) * omega;
      ax = -drag * norm * mx + lift * norm * sy * mz;
      ay = -drag * norm * my - lift * norm * sx * mz;
      az = -G - drag * norm * mz + lift * norm * (sx * my - sy * mx);
      x += mx * STEP;
      y += my * STEP;
      z += mz * STEP;
      vx += ax * STEP;
      vy += ay * STEP;
      vz += az * STEP;
      if (oldZ > HUB_HEIGHT && z <= HUB_HEIGHT && vz < 0) {
        double f = (oldZ - HUB_HEIGHT) / (oldZ - z);
        return new Flight(oldX + f * (x - oldX), oldY + f * (y - oldY), t + f * STEP);
      }
      if (z < 0 || !Double.isFinite(x + y + z + vx + vy + vz)) return null;
    }
    return null;
  }

  private static boolean finite(double... values) {
    for (double v : values) if (!Double.isFinite(v)) return false;
    return true;
  }

  private static void check(boolean value, String reason) {
    if (!value) throw new IllegalArgumentException(reason);
  }
}
