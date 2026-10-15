import { rad, finite, type Physics, type Vec, type Scenario } from "./types";
export interface Flight {
  path: Vec[];
  times: number[];
  scored: boolean;
  collision: boolean;
  crossing: Vec | null;
  tof: number;
  final: Vec;
}
const add = (a: Vec, b: Vec, k = 1): Vec => [
  a[0] + b[0] * k,
  a[1] + b[1] * k,
  a[2] + b[2] * k,
];
const cross = (a: Vec, b: Vec): Vec => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export function physicsAtBearing(p: Physics, azimuth: number): Physics {
  const [wx, wy, wz] = p.wind,
    c = Math.cos(azimuth),
    s = Math.sin(azimuth);
  return { ...p, wind: [c * wx + s * wy, -s * wx + c * wy, wz] };
}
export function acceleration(v: Vec, spin: Vec, p: Physics): Vec {
  const air = add(v, p.wind, -1),
    speed = Math.hypot(...air),
    area = Math.PI * p.radius ** 2;
  const drag = air.map(
    (x) => (-0.5 * p.density * p.drag * area * speed * x) / p.mass,
  ) as Vec;
  const lift = cross(spin, air).map(
    (x) => (0.5 * p.density * area * p.magnus * p.radius * speed * x) / p.mass,
  ) as Vec;
  return [drag[0] + lift[0], drag[1] + lift[1], drag[2] + lift[2] - p.gravity];
}
export function simulate(
  p: Physics,
  s: Scenario,
  angle: number,
  speed: number,
  spinRps: number,
  yaw = 0,
  until?: number,
  spinAxis?: Vec,
): Flight {
  if (
    !finite(angle, speed, spinRps, yaw, s.distance, s.radial, s.tangential) ||
    p.dt <= 0 ||
    p.mass <= 0
  )
    throw Error("Invalid flight inputs");
  const e = rad(angle),
    c = speed * Math.cos(e);
  let pos: Vec = [0, 0, p.launchHeight],
    vel: Vec = [
      c * Math.cos(yaw) + s.radial,
      c * Math.sin(yaw) + s.tangential,
      speed * Math.sin(e),
    ];
  const axis: Vec = spinAxis ?? [Math.sin(yaw), -Math.cos(yaw), 0];
  const path: Vec[] = [pos],
    times = [0];
  let collision = false,
    crossing: Vec | null = null,
    tof = 0,
    scored = false;
  const end = until ?? 5;
  let t = 0;
  while (t < end - 1e-9) {
    const h = Math.min(
        p.dt,
        (0.4 * (p.radius + p.rimRadius)) / Math.max(1, Math.hypot(...vel)),
        end - t,
      ),
      prev = pos;
    const a = (v: Vec, time: number) =>
      acceleration(
        v,
        axis.map(
          (x) => x * spinRps * 2 * Math.PI * Math.exp(-p.spinDecay * time),
        ) as Vec,
        p,
      );
    const k1 = a(vel, t),
      v2 = add(vel, k1, h / 2),
      k2 = a(v2, t + h / 2),
      v3 = add(vel, k2, h / 2),
      k3 = a(v3, t + h / 2),
      v4 = add(vel, k3, h),
      k4 = a(v4, t + h);
    pos = pos.map(
      (x, i) => x + (h * (vel[i] + 2 * v2[i] + 2 * v3[i] + v4[i])) / 6,
    ) as Vec;
    vel = vel.map(
      (x, i) => x + (h * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])) / 6,
    ) as Vec;
    t += h;
    path.push(pos);
    times.push(t);
    if (until !== undefined) continue;
    const dx = pos[0] - s.distance,
      radius = Math.hypot(dx, pos[1]);
    if (
      Math.hypot(radius - p.hubRadius, pos[2] - p.hubHeight) <=
      p.radius + p.rimRadius
    ) {
      collision = true;
      // A rim touch is excluded from the robust region. Reflect for the displayed bounce.
      const dr = radius - p.hubRadius,
        dz = pos[2] - p.hubHeight,
        norm = Math.hypot(dr, dz) || 1;
      const n: Vec = [
        ((dr / norm) * dx) / (radius || 1),
        ((dr / norm) * pos[1]) / (radius || 1),
        dz / norm,
      ];
      const vn = vel.reduce((sum, x, i) => sum + x * n[i], 0);
      if (vn < 0) vel = add(vel, n, -(1 + p.restitution) * vn);
    }
    if (
      !crossing &&
      prev[2] > p.hubHeight &&
      pos[2] <= p.hubHeight &&
      vel[2] < 0
    ) {
      const f = (prev[2] - p.hubHeight) / (prev[2] - pos[2]);
      crossing = prev.map((x, i) => x + f * (pos[i] - x)) as Vec;
      tof = t - h + f * h;
    }
    if (crossing && pos[2] < p.hubHeight - p.radius - p.rimRadius) {
      scored =
        !collision &&
        Math.hypot(crossing[0] - s.distance, crossing[1]) +
          p.radius +
          p.rimRadius <
          p.hubRadius;
      break;
    }
    if (pos[2] < p.radius || !finite(...pos, ...vel)) break;
  }
  return { path, times, scored, collision, crossing, tof, final: pos };
}
export function tiltInverse(v: Vec, roll: number, pitch: number): Vec {
  // Inverse of Rz(yaw) Ry(pitch) Rx(roll), with yaw already removed.
  const cp = Math.cos(pitch),
    sp = Math.sin(pitch),
    cr = Math.cos(roll),
    sr = Math.sin(roll);
  const x = cp * v[0] - sp * v[2],
    z = sp * v[0] + cp * v[2];
  return [x, cr * v[1] + sr * z, -sr * v[1] + cr * z];
}
export function turretState(
  x: number,
  y: number,
  heading: number,
  vx: number,
  vy: number,
  omega: number,
  offset: [number, number],
  latency: number,
) {
  const yaw = heading + omega * latency,
    c = Math.cos(yaw),
    s = Math.sin(yaw),
    ox = c * offset[0] - s * offset[1],
    oy = s * offset[0] + c * offset[1];
  return {
    x: x + vx * latency + ox,
    y: y + vy * latency + oy,
    vx: vx - omega * oy,
    vy: vy + omega * ox,
    heading: yaw,
  };
}

/** Allocation-light RK4 for offline searches. Rim touches are rejected without following the bounce. */
export function scoringFlight(
  p: Physics,
  s: Scenario,
  angle: number,
  speed: number,
  spinRps: number,
  yaw = 0,
  axisOverride?: Vec,
): Flight {
  const elevation = rad(angle),
    horizontal = speed * Math.cos(elevation);
  let x = 0,
    y = 0,
    z = p.launchHeight,
    vx = horizontal * Math.cos(yaw) + s.radial,
    vy = horizontal * Math.sin(yaw) + s.tangential,
    vz = speed * Math.sin(elevation),
    t = 0;
  const axis = axisOverride ?? [Math.sin(yaw), -Math.cos(yaw), 0],
    spin = 2 * Math.PI * spinRps;
  const drag =
    (0.5 * p.density * Math.PI * p.radius * p.radius * p.drag) / p.mass;
  const magnus =
    (0.5 * p.density * Math.PI * p.radius * p.radius * p.magnus * p.radius) /
    p.mass;
  let collision = false,
    crossing: Vec | null = null,
    tof = 0;
  const accelerationAt = (
    a: number,
    b: number,
    c: number,
    time: number,
  ): Vec => {
    const ux = a - p.wind[0],
      uy = b - p.wind[1],
      uz = c - p.wind[2],
      norm = Math.hypot(ux, uy, uz),
      omega = spin * Math.exp(-p.spinDecay * time),
      lift = magnus * norm * omega;
    return [
      -drag * norm * ux + lift * (axis[1] * uz - axis[2] * uy),
      -drag * norm * uy + lift * (axis[2] * ux - axis[0] * uz),
      -drag * norm * uz + lift * (axis[0] * uy - axis[1] * ux) - p.gravity,
    ];
  };
  while (t < 5) {
    const h = Math.min(
        p.dt,
        (0.4 * (p.radius + p.rimRadius)) / Math.max(1, Math.hypot(vx, vy, vz)),
      ),
      oldX = x,
      oldY = y,
      oldZ = z;
    const a = accelerationAt(vx, vy, vz, t),
      bx = vx + (a[0] * h) / 2,
      by = vy + (a[1] * h) / 2,
      bz = vz + (a[2] * h) / 2;
    const b = accelerationAt(bx, by, bz, t + h / 2),
      cx = vx + (b[0] * h) / 2,
      cy = vy + (b[1] * h) / 2,
      cz = vz + (b[2] * h) / 2;
    const c = accelerationAt(cx, cy, cz, t + h / 2),
      dx = vx + c[0] * h,
      dy = vy + c[1] * h,
      dz = vz + c[2] * h,
      d = accelerationAt(dx, dy, dz, t + h);
    x += (h * (vx + 2 * bx + 2 * cx + dx)) / 6;
    y += (h * (vy + 2 * by + 2 * cy + dy)) / 6;
    z += (h * (vz + 2 * bz + 2 * cz + dz)) / 6;
    vx += (h * (a[0] + 2 * b[0] + 2 * c[0] + d[0])) / 6;
    vy += (h * (a[1] + 2 * b[1] + 2 * c[1] + d[1])) / 6;
    vz += (h * (a[2] + 2 * b[2] + 2 * c[2] + d[2])) / 6;
    t += h;
    const radius = Math.hypot(x - s.distance, y);
    if (
      Math.hypot(radius - p.hubRadius, z - p.hubHeight) <=
      p.radius + p.rimRadius
    )
      collision = true;
    if (!crossing && oldZ > p.hubHeight && z <= p.hubHeight && vz < 0) {
      const f = (oldZ - p.hubHeight) / (oldZ - z);
      crossing = [oldX + f * (x - oldX), oldY + f * (y - oldY), p.hubHeight];
      tof = t - h + f * h;
    }
    if (crossing && z < p.hubHeight - p.radius - p.rimRadius) break;
    if (z < p.radius || !Number.isFinite(x + y + z + vx + vy + vz)) break;
  }
  const scored =
    !!crossing &&
    !collision &&
    Math.hypot(crossing[0] - s.distance, crossing[1]) + p.radius + p.rimRadius <
      p.hubRadius;
  return {
    path: [],
    times: [],
    scored,
    collision,
    crossing,
    tof,
    final: [x, y, z],
  };
}
