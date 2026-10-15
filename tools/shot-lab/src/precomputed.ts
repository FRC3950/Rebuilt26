import { leastSquares } from "./calibration";
import { scoringFlight } from "./physics";
import { defaults, type Physics, type Geometry } from "./types";

// Version 2 is deliberately radial-only. No tangential or bearing coefficient layers.
export interface MapConfig {
  height: number;
  radius: number;
  angleMin: number;
  angleMax: number;
  transfer: number;
  drag: number;
  spinRatio: number;
  lift: number;
  spinDecay: number;
  sigmaAngle: number;
  sigmaSpeed: number;
}
export interface Band {
  angle: number;
  lower: number;
  upper: number;
}
export interface MapShot {
  values: number[];
  score: number;
}
export interface MapPatch {
  bounds: number[];
  center: number[];
  scale: number[];
  coefficients: number[][];
  maxErrorFraction: number;
}
export interface RadialMap {
  version: 2;
  reference: { physics: Physics; geometry: Geometry };
  basis: string[];
  units: {
    distance: string;
    radial: string;
    angle: string;
    speed: string;
    tof: string;
  };
  kind: "radial-quadratic";
  calibrated: boolean;
  config: MapConfig;
  resolution: { angle: number; speed: number };
  patches: MapPatch[];
  gaps: number[][];
  validation: {
    samples: number;
    failures: number;
    motionLimitFailures: number;
    maxErrorFraction: number;
    physicalValidation: false;
  };
  domain: number[];
  coverage?: {
    total: number;
    passed: number;
    uncovered: number;
    mechanismLimitFailures: number;
    simulatedMisses: number;
    description: string;
    physicalValidation: false;
    points: { x: number; y: number; tested: number; passed: number }[];
  };
}
export const baseline: MapConfig = {
  height: 0.445,
  radius: 0.0508,
  angleMin: 77,
  angleMax: 60.15,
  transfer: 1,
  drag: 0,
  spinRatio: 0,
  lift: 0,
  spinDecay: 0,
  sigmaAngle: 0.5,
  sigmaSpeed: 0.15,
};
export const terms = (d: number, r: number) => [1, d, r, d * d, d * r, r * r];
export function evaluate(m: RadialMap, d: number, r: number) {
  const p = m.patches.find(
    (p) =>
      d >= p.bounds[0] &&
      d <= p.bounds[1] &&
      r >= p.bounds[2] &&
      r <= p.bounds[3],
  );
  if (!p)
    throw Error("Unvalidated map region: regenerate or inspect coverage gaps");
  const b = terms(
    (d - p.center[0]) / p.scale[0],
    (r - p.center[1]) / p.scale[1],
  );
  return p.coefficients.map((c) => c.reduce((s, v, i) => s + v * b[i], 0));
}
export function compensate(angle: number, speed: number, tangential: number) {
  const a = (angle * Math.PI) / 180,
    x = speed * Math.cos(a),
    y = -tangential,
    z = speed * Math.sin(a);
  return {
    speed: Math.hypot(x, y, z),
    angle: (Math.atan2(z, Math.hypot(x, y)) * 180) / Math.PI,
    yaw: Math.atan2(y, x),
    vector: [x, y, z],
  };
}
function physics(c: MapConfig): Physics {
  return {
    ...defaults().physics,
    launchHeight: c.height,
    drag: c.drag,
    magnus: c.lift,
    spinDecay: c.spinDecay,
    wind: [0, 0, 0],
    dt: 0.01,
  };
}
export function flight(
  c: MapConfig,
  d: number,
  r: number,
  a: number,
  v: number,
  tangential = 0,
) {
  const q = compensate(a, v, tangential),
    spin = (c.spinRatio * q.speed) / (2 * Math.PI * c.radius * c.transfer);
  return scoringFlight(
    physics(c),
    { distance: d, radial: r, tangential },
    q.angle,
    q.speed,
    spin,
    q.yaw,
  );
}
// Analytic vacuum trajectory with continuous torus-clearance minimization, including ascending entry.
function vacuum(c: MapConfig, d: number, r: number, a: number, v: number) {
  const e = (a * Math.PI) / 180,
    z = v * Math.sin(e),
    x = v * Math.cos(e) + r,
    g = 9.81,
    h = 1.83,
    R = 0.075 + 0.025;
  const disc = z * z - 2 * g * (h - c.height);
  if (disc <= 0 || x <= 0) return { scored: false, tof: 0 };
  const t = (z + Math.sqrt(disc)) / g;
  if (Math.abs(x * t - d) >= 0.56 - R) return { scored: false, tof: t };
  const clearance = (time: number) =>
    Math.hypot(
      Math.abs(x * time - d) - 0.56,
      c.height + z * time - 0.5 * g * time * time - h,
    );
  for (const sign of [-1, 1]) {
    const root = (level: number) => {
      const k = z * z - 2 * g * (level - c.height);
      return k < 0 ? z / g : (z + sign * Math.sqrt(k)) / g;
    };
    let lo = Math.max(0, Math.min(root(h - R), root(h + R))),
      hi = Math.max(root(h - R), root(h + R));
    // Split at the hub center because the radial absolute value has a cusp.
    const intervals = [lo, ...(d / x > lo && d / x < hi ? [d / x] : []), hi];
    for (let i = 0; i < intervals.length - 1; i++) {
      let l = intervals[i],
        u = intervals[i + 1];
      for (let j = 0; j < 12; j++) {
        const p = l + (u - l) / 3,
          q = u - (u - l) / 3;
        if (clearance(p) < clearance(q)) u = q;
        else l = p;
      }
      if (clearance((l + u) / 2) <= R + 0.001) return { scored: false, tof: t };
    }
  }
  return { scored: true, tof: t };
}
export function sweep(
  c: MapConfig,
  d: number,
  r: number,
  angleStep = 0.5,
  speedStep = 0.15,
) {
  if (
    !Object.values(c).every(Number.isFinite) ||
    c.sigmaAngle <= 0 ||
    c.sigmaSpeed <= 0 ||
    c.transfer <= 0 ||
    c.radius <= 0 ||
    c.radius > 0.3 ||
    c.transfer > 2 ||
    !Number.isFinite(d) ||
    !Number.isFinite(r) ||
    !Number.isFinite(angleStep) ||
    angleStep <= 0 ||
    !Number.isFinite(speedStep) ||
    speedStep <= 0 ||
    c.height <= 0 ||
    c.height >= 1.83 ||
    Math.min(c.angleMin, c.angleMax) < 5 ||
    Math.max(c.angleMin, c.angleMax) > 88 ||
    Math.abs(c.angleMin - c.angleMax) < 1 ||
    c.drag < 0 ||
    c.drag > 2 ||
    Math.abs(c.spinRatio) > 10 ||
    c.lift < 0 ||
    c.lift > 2 ||
    c.spinDecay < 0 ||
    c.spinDecay > 10
  )
    throw Error("Invalid physical inputs or variability");
  const amin = Math.min(c.angleMin, c.angleMax),
    amax = Math.max(c.angleMin, c.angleMax),
    vmax = 100 * 2 * Math.PI * c.radius * c.transfer;
  const test = (a: number, v: number) =>
    c.drag === 0 && (c.lift === 0 || c.spinRatio === 0)
      ? vacuum(c, d, r, a, v)
      : flight(c, d, r, a, v);
  const bands: Band[] = [],
    candidates: { a: number; v: number; tof: number }[] = [];
  const na = Math.ceil((amax - amin) / angleStep),
    nv = Math.ceil(vmax / speedStep);
  // Enumerate the entire sampled command domain, keeping every disconnected scoring interval.
  for (let i = 0; i <= na; i++) {
    const a = amin + ((amax - amin) * i) / na;
    let start: number | null = null;
    for (let j = 0; j <= nv + 1; j++) {
      const v = (vmax * j) / nv,
        ok = j <= nv && test(a, v).scored;
      if (ok && start === null) start = v;
      if (!ok && start !== null) {
        let lower = start,
          upper = (vmax * (j - 1)) / nv;
        const edge = (good: number, bad: number) => {
          for (let k = 0; k < 10; k++) {
            const mid = (good + bad) / 2;
            if (test(a, mid).scored) good = mid;
            else bad = mid;
          }
          return good;
        };
        lower = edge(lower, Math.max(0, lower - vmax / nv));
        upper = edge(upper, Math.min(vmax, upper + vmax / nv));
        bands.push({ angle: a, lower, upper });
        for (let k = 1; k <= 5; k++) {
          const speed = lower + ((upper - lower) * k) / 6;
          candidates.push({ a, v: speed, tof: test(a, speed).tof });
        }
        start = null;
      }
    }
  }
  let best: MapShot | null = null;
  for (const q of candidates) {
    const b = bands.find(
      (b) => b.angle === q.a && q.v >= b.lower && q.v <= b.upper,
    )!;
    const margin = (sign: number) => {
      let good = 0,
        bad = sign < 0 ? q.a - amin : amax - q.a;
      for (let i = 0; i < 12; i++) {
        const m = (good + bad) / 2;
        if (test(q.a + sign * m, q.v).scored) good = m;
        else bad = m;
      }
      return good;
    };
    const m = [margin(-1), margin(1), q.v - b.lower, b.upper - q.v];
    const score = Math.min(
      m[0] / c.sigmaAngle,
      m[1] / c.sigmaAngle,
      m[2] / c.sigmaSpeed,
      m[3] / c.sigmaSpeed,
    );
    if (!best || score > best.score)
      best = { values: [q.a, q.v, q.tof, ...m], score };
  }
  // Refine the robust winner continuously; otherwise grid jumps poison quadratic fits.
  if (best) {
    const assess = (a: number, v: number): MapShot | null => {
      if (a <= amin || a >= amax || v <= 0 || v >= vmax || !test(a, v).scored)
        return null;
      const margin = (axis: number, sign: number) => {
        let good = 0,
          bad =
            axis === 0
              ? sign < 0
                ? a - amin
                : amax - a
              : sign < 0
                ? v
                : vmax - v;
        for (let i = 0; i < 14; i++) {
          const m = (good + bad) / 2;
          if (
            test(
              a + (axis === 0 ? sign * m : 0),
              v + (axis === 1 ? sign * m : 0),
            ).scored
          )
            good = m;
          else bad = m;
        }
        return good;
      };
      const m = [margin(0, -1), margin(0, 1), margin(1, -1), margin(1, 1)];
      return {
        values: [a, v, test(a, v).tof, ...m],
        score: Math.min(
          m[0] / c.sigmaAngle,
          m[1] / c.sigmaAngle,
          m[2] / c.sigmaSpeed,
          m[3] / c.sigmaSpeed,
        ),
      };
    };
    best = assess(best.values[0], best.values[1]) ?? best;
    let da = angleStep,
      dv = speedStep;
    for (let k = 0; k < 12; k++) {
      const previous: MapShot = best;
      for (const x of [-1, 0, 1])
        for (const y of [-1, 0, 1]) {
          const q = assess(
            previous.values[0] + x * da,
            previous.values[1] + y * dv,
          );
          if (q && q.score > best.score) best = q;
        }
      if (best === previous) {
        da /= 2;
        dv /= 2;
      }
    }
  }
  return { bands, best };
}
export async function generate(
  c: MapConfig,
  domain = [0.65, 7.5, -4.25, 4.25],
  progress: (s: string) => void = () => {},
  cancel: () => boolean = () => false,
): Promise<RadialMap> {
  const model: RadialMap = {
    version: 2,
    reference: { physics: physics(c), geometry: defaults().geometry },
    basis: ["1", "d", "r", "d*d", "d*r", "r*r"],
    units: {
      distance: "m",
      radial: "m/s",
      angle: "deg",
      speed: "m/s",
      tof: "s",
    },
    kind: "radial-quadratic",
    calibrated: false,
    config: { ...c },
    resolution: { angle: 0.5, speed: 0.15 },
    patches: [],
    gaps: [],
    validation: {
      samples: 0,
      failures: 0,
      motionLimitFailures: 0,
      maxErrorFraction: 0,
      physicalValidation: false,
    },
    domain,
  };
  const cache = new Map<string, MapShot | null>();
  const sample = (d: number, r: number) => {
    const key = [d, r].map((x) => x.toFixed(7)).join(",");
    if (!cache.has(key)) cache.set(key, sweep(c, d, r).best);
    return cache.get(key)!;
  };
  async function region(bounds: number[], depth: number) {
    if (cancel()) throw Error("Cancelled");
    const [d0, d1, r0, r1] = bounds,
      center = [(d0 + d1) / 2, (r0 + r1) / 2],
      scale = [(d1 - d0) / 2, (r1 - r0) / 2];
    const rows: number[][] = [],
      ys: number[][] = [];
    let missing = false;
    for (const x of [-1, 0, 1])
      for (const y of [-1, 0, 1]) {
        const s = sample(center[0] + scale[0] * x, center[1] + scale[1] * y);
        if (!s) {
          missing = true;
          continue;
        }
        rows.push(terms(x, y));
        ys.push(s.values);
      }
    const p: MapPatch = {
      bounds,
      center,
      scale,
      coefficients: [],
      maxErrorFraction: 0,
    };
    if (!missing) {
      p.coefficients = Array.from({ length: 7 }, (_, i) =>
        leastSquares(
          rows,
          ys.map((y) => y[i]),
        ),
      );
      for (const x of [-1, -0.5, 0, 0.5, 1])
        for (const y of [-1, -0.5, 0, 0.5, 1]) {
          const d = center[0] + scale[0] * x,
            r = center[1] + scale[1] * y,
            s = sample(d, r);
          model.validation.samples++;
          if (!s) {
            missing = true;
            continue;
          }
          const b = terms(x, y),
            v = p.coefficients.map((c) =>
              c.reduce((sum, k, i) => sum + k * b[i], 0),
            );
          const scales = [
            Math.min(s.values[3], s.values[4]),
            Math.min(s.values[5], s.values[6]),
            0.1,
            ...s.values.slice(3),
          ];
          const error = Math.max(
            ...v.map(
              (k, i) => Math.abs(k - s.values[i]) / Math.max(1e-5, scales[i]),
            ),
          );
          p.maxErrorFraction = Math.max(p.maxErrorFraction, error);
          const lateralLimit = Math.sqrt(Math.max(0, 4.25 ** 2 - r ** 2));
          for (const tangential of [
            -lateralLimit,
            -lateralLimit / 2,
            0,
            lateralLimit / 2,
            lateralLimit,
          ]) {
            const compensated = compensate(v[0], v[1], tangential);
            if (
              compensated.angle < Math.min(c.angleMin, c.angleMax) ||
              compensated.angle > Math.max(c.angleMin, c.angleMax) ||
              compensated.speed > 100 * 2 * Math.PI * c.radius * c.transfer
            )
              model.validation.motionLimitFailures++;
            if (!flight(c, d, r, v[0], v[1], tangential).scored) missing = true;
          }
        }
    }
    if (!missing && p.maxErrorFraction < 0.1) {
      model.patches.push(p);
      model.validation.maxErrorFraction = Math.max(
        model.validation.maxErrorFraction,
        p.maxErrorFraction,
      );
    } else if (depth < 3 && rows.length > 0) {
      const [dm, rm] = center;
      for (const b of [
        [d0, dm, r0, rm],
        [dm, d1, r0, rm],
        [d0, dm, rm, r1],
        [dm, d1, rm, r1],
      ])
        await region(b, depth + 1);
    } else {
      model.gaps.push(bounds);
      model.validation.failures++;
    }
    progress(
      `${cache.size} trajectories optimized · ${model.patches.length} accepted patches · ${model.gaps.length} gaps`,
    );
    await new Promise<void>((resolve) => setTimeout(resolve, 0));
  }
  // Small initial cells keep normalized quadratic fits local and bounded.
  for (let d = domain[0]; d < domain[1] - 1e-6; d += 0.8)
    for (let r = domain[2]; r < domain[3] - 1e-6; r += 1)
      await region(
        [d, Math.min(domain[1], d + 0.8), r, Math.min(domain[3], r + 1)],
        0,
      );
  return model;
}
