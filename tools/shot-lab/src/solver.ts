import { mapValue, conditionalMap, leastSquares } from "./calibration";
import {
  turretState,
  tiltInverse,
  physicsAtBearing,
  scoringFlight,
} from "./physics";
import {
  rad,
  deg,
  finite,
  type Project,
  type Scenario,
  type Shot,
  type Model,
  type Patch,
  type CoveragePoint,
} from "./types";
export function shotPhysics(p: Project, azimuth: number) {
  return physicsAtBearing(p.physics, azimuth);
}

export function releaseRps(p: Project, speed: number, angle: number) {
  const hood = mapValue(
    p.calibration.angle,
    angle - p.physics.angleOffset,
    true,
  );
  return mapValue(
    conditionalMap(p.calibration.speedByHood, hood, p.calibration.speed),
    speed / p.physics.speedScale,
    true,
  );
}
export function spinAt(p: Project, speed: number, angle: number) {
  const hood = mapValue(
    p.calibration.angle,
    angle - p.physics.angleOffset,
    true,
  );
  return (
    mapValue(
      conditionalMap(p.calibration.spinByHood, hood, p.calibration.spin),
      releaseRps(p, speed, angle),
    ) * p.physics.spinScale
  );
}
function speedBounds(p: Project, angle: number) {
  const hood = mapValue(
      p.calibration.angle,
      angle - p.physics.angleOffset,
      true,
    ),
    points = conditionalMap(
      p.calibration.speedByHood,
      hood,
      p.calibration.speed,
    );
  return p.geometry.rps.map((x) =>
    Math.max(0.1, mapValue(points, x) * p.physics.speedScale),
  );
}
export function launchBounds(p: Project) {
  const a = p.geometry.hood
    .map((x) => mapValue(p.calibration.angle, x) + p.physics.angleOffset)
    .sort((a, b) => a - b);
  const v = p.geometry.rps
    .map((x) => mapValue(p.calibration.speed, x) * p.physics.speedScale)
    .sort((a, b) => a - b);
  return { angle: a, speed: [Math.max(0.1, v[0]), v[1]] };
}
export function trace(
  p: Project,
  s: Scenario,
  angle: number,
  speed: number,
  yaw?: number,
  spinAxis?: [number, number, number],
) {
  const h = speed * Math.cos(rad(angle));
  const base = Math.asin(
    Math.max(-0.99, Math.min(0.99, -s.tangential / Math.max(0.1, h))),
  );
  return scoringFlight(
    shotPhysics(p, s.azimuth ?? 0),
    s,
    angle,
    speed,
    spinAt(p, speed, angle),
    yaw ?? base,
    spinAxis,
  );
}
function centerFlight(p: Project, s: Scenario, angle: number, speed: number) {
  let yaw = Math.asin(
    Math.max(
      -0.95,
      Math.min(
        0.95,
        -s.tangential / Math.max(0.1, speed * Math.cos(rad(angle))),
      ),
    ),
  );
  for (let i = 0; i < 3; i++) {
    const f = trace(p, s, angle, speed, yaw);
    if (!f.crossing) break;
    yaw -= Math.atan2(f.crossing[1], Math.max(0.2, f.crossing[0]));
  }
  return { flight: trace(p, s, angle, speed, yaw), yaw };
}
export interface Candidate {
  angle: number;
  speed: number;
  scored: boolean;
  minimumSpeed?: number;
  maximumSpeed?: number;
}
export function solveShot(
  p: Project,
  s: Scenario,
  detail?: Candidate[],
): Shot | null {
  const limits = launchBounds(p);
  const success = (a: number, v: number) => {
    const b = speedBounds(p, a);
    return v >= b[0] && v <= b[1] && centerFlight(p, s, a, v).flight.scored;
  };
  function candidate(angle: number): Shot | null {
    const bounds = speedBounds(p, angle);
    let lo = bounds[0],
      hi = bounds[1];
    for (let j = 0; j < 18; j++) {
      const v = (lo + hi) / 2,
        f = centerFlight(p, s, angle, v).flight;
      if ((f.crossing?.[0] ?? f.final[0]) < s.distance) lo = v;
      else hi = v;
    }
    const speed = (lo + hi) / 2,
      center = centerFlight(p, s, angle, speed);
    const dot: Candidate = { angle, speed, scored: center.flight.scored };
    detail?.push(dot);
    if (!center.flight.scored) return null;
    function margin(axis: "angle" | "speed", sign: number) {
      const value = axis === "angle" ? angle : speed,
        b = axis === "angle" ? limits.angle : bounds;
      let low = 0,
        high = Math.abs((sign < 0 ? b[0] : b[1]) - value);
      for (let j = 0; j < 13; j++) {
        const m = (low + high) / 2;
        if (
          success(
            axis === "angle" ? angle + sign * m : angle,
            axis === "speed" ? speed + sign * m : speed,
          )
        )
          low = m;
        else high = m;
      }
      return low;
    }
    const margins = [
      margin("angle", -1),
      margin("angle", 1),
      margin("speed", -1),
      margin("speed", 1),
    ];
    dot.minimumSpeed = speed - margins[2];
    dot.maximumSpeed = speed + margins[3];
    const score = Math.min(
      margins[0] / p.calibration.sigmaAngle,
      margins[1] / p.calibration.sigmaAngle,
      margins[2] / p.calibration.sigmaSpeed,
      margins[3] / p.calibration.sigmaSpeed,
    );
    return {
      angle,
      speed,
      tof: center.flight.tof,
      yaw: center.yaw,
      margins,
      score,
    };
  }
  let best: Shot | null = null;
  const step = (limits.angle[1] - limits.angle[0]) / 18;
  for (let i = 0; i <= 18; i++) {
    const q = candidate(limits.angle[0] + step * i);
    if (q && (!best || q.score > best.score)) best = q;
  }
  if (!best) return null;
  // Refine the selected branch continuously so fits do not inherit coarse angle-grid jumps.
  let a = Math.max(limits.angle[0], best.angle - step),
    b = Math.min(limits.angle[1], best.angle + step);
  for (let i = 0; i < 10; i++) {
    const left = a + (b - a) / 3,
      right = b - (b - a) / 3,
      l = candidate(left),
      r = candidate(right);
    for (const q of [l, r]) if (q && q.score > best.score) best = q;
    if ((l?.score ?? -1) > (r?.score ?? -1)) b = right;
    else a = left;
  }
  return best;
}
export function basis(d: number, r: number) {
  return [1, d, r, d * d, d * r, r * r];
}
export function evaluatePatch(p: Patch, d: number, r: number) {
  const b = basis(
    (d - p.center[0]) / p.scale[0],
    (r - p.center[1]) / p.scale[1],
  );
  const dot = (c: number[]) => c.reduce((sum, x, i) => sum + x * b[i], 0);
  return { values: p.coefficients.map(dot), yaw: dot(p.yaw) };
}
export function evaluateModel(m: Model, s: Scenario): Shot {
  const directions = [...new Set(m.patches.map((p) => p.azimuth ?? 0))].sort(
    (a, b) => a - b,
  );
  const phi =
    (((s.azimuth ?? 0) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
  function direction(azimuth: number) {
    const patches = m.patches.filter((p) => (p.azimuth ?? 0) === azimuth);
    const layers = [...new Set(patches.map((p) => p.tangential))].sort(
        (a, b) => a - b,
      ),
      t = s.tangential;
    let k = 0;
    while (k < layers.length - 2 && t > layers[k + 1]) k++;
    if (!layers.length || t < layers[0] - 1e-6 || t > layers.at(-1)! + 1e-6)
      throw Error("Motion coverage missing");
    function layer(v: number) {
      const p = patches.find(
        (p) =>
          p.tangential === v &&
          s.distance >= p.bounds[0] - 1e-6 &&
          s.distance <= p.bounds[1] + 1e-6 &&
          s.radial >= p.bounds[2] - 1e-6 &&
          s.radial <= p.bounds[3] + 1e-6,
      );
      if (!p) throw Error("Fit coverage missing");
      return evaluatePatch(p, s.distance, s.radial);
    }
    const a = layer(layers[k]),
      b = layer(layers[Math.min(k + 1, layers.length - 1)]),
      f =
        layers.length === 1 ? 0 : (t - layers[k]) / (layers[k + 1] - layers[k]);
    return {
      values: a.values.map((x, i) => x + (b.values[i] - x) * f),
      yaw: a.yaw + (b.yaw - a.yaw) * f,
    };
  }
  let index = 0;
  while (index < directions.length - 1 && phi > directions[index + 1]) index++;
  const next = (index + 1) % directions.length,
    a = direction(directions[index]),
    b = direction(directions[next]);
  const gap =
      (directions[next] - directions[index] + 2 * Math.PI) % (2 * Math.PI),
    f = directions.length === 1 ? 0 : (phi - directions[index]) / gap;
  const v = a.values.map((x, i) => x + (b.values[i] - x) * f),
    yaw = a.yaw + (b.yaw - a.yaw) * f;
  if (
    !finite(...v, yaw) ||
    v[1] <= 0 ||
    v[2] <= 0 ||
    v.slice(3).some((x) => x <= 0)
  )
    throw Error("Non-finite or invalid polynomial output");
  return {
    angle: v[0],
    speed: v[1],
    tof: v[2],
    margins: v.slice(3),
    yaw,
    score: 0,
  };
}
function fitPatch(
  samples: { s: Scenario; shot: Shot }[],
  bounds: Patch["bounds"],
  tangential: number,
  azimuth: number,
): Patch {
  const center: [number, number] = [
      (bounds[0] + bounds[1]) / 2,
      (bounds[2] + bounds[3]) / 2,
    ],
    scale: [number, number] = [
      (bounds[1] - bounds[0]) / 2,
      (bounds[3] - bounds[2]) / 2,
    ];
  const x = samples.map(({ s }) =>
    basis(
      (s.distance - center[0]) / scale[0],
      (s.radial - center[1]) / scale[1],
    ),
  );
  const output = samples.map(({ shot: q }) => [
    q.angle,
    q.speed,
    q.tof,
    ...q.margins,
  ]);
  return {
    bounds,
    center,
    scale,
    tangential,
    azimuth,
    coefficients: Array.from({ length: 7 }, (_, i) =>
      leastSquares(
        x,
        output.map((o) => o[i]),
      ),
    ),
    yaw: leastSquares(
      x,
      samples.map((x) => x.shot.yaw),
    ),
  };
}
export const emptyValidation = () => ({
  passed: false,
  samples: 0,
  failures: 0,
  maxMarginFraction: 0,
  physicalValidated: false,
  residuals: [],
  coverage: [] as CoveragePoint[],
  issues: [] as string[],
});
export async function generateModel(
  p: Project,
  progress: (message: string, fraction: number) => void,
): Promise<Model> {
  if (!p.calibration.measured || !p.calibration.aeroFitted)
    throw Error(
      "Measured transfer calibration and identifiable aerodynamic flight fit are required.",
    );
  const g = p.geometry,
    offset = Math.max(...g.turrets.map((t) => Math.hypot(...t))),
    maxMotion = g.maxSpeed + g.maxOmega * offset;
  const maxD =
    Math.max(
      ...[g.zone[0], g.zone[1]].flatMap((x) =>
        [g.zone[2], g.zone[3]].map((y) =>
          Math.hypot(x - g.hub[0], y - g.hub[1]),
        ),
      ),
    ) +
    offset +
    g.maxSpeed * p.physics.latency;
  const centerDistances: number[] = [];
  for (let i = 0; i <= 16; i++)
    for (let j = 0; j <= 24; j++) {
      const x = g.zone[0] + ((g.zone[1] - g.zone[0]) * i) / 16,
        y = g.zone[2] + ((g.zone[3] - g.zone[2]) * j) / 24;
      if (Math.abs(x - g.hub[0]) >= 1.05 || Math.abs(y - g.hub[1]) >= 1.05)
        centerDistances.push(Math.hypot(x - g.hub[0], y - g.hub[1]));
    }
  if (!centerDistances.length)
    throw Error("The selected zone contains no navigable field samples.");
  const minD = Math.max(
    0.05,
    Math.min(...centerDistances) - offset - g.maxSpeed * p.physics.latency,
  );
  const model: Model = {
    version: 1,
    name: p.name,
    createdAt: new Date().toISOString(),
    units: "m, s, deg, m/s, rps",
    physics: p.physics,
    geometry: g,
    calibration: p.calibration,
    patches: [],
    validation: emptyValidation(),
  };
  let visited = 0;
  const cache = new Map<string, Shot | null>();
  function optimal(s: Scenario) {
    const key = [s.distance, s.radial, s.tangential, s.azimuth ?? 0]
      .map((x) => x.toFixed(9))
      .join(",");
    if (!cache.has(key)) cache.set(key, solveShot(p, s));
    return cache.get(key)!;
  }

  async function patch(
    bounds: Patch["bounds"],
    t: number,
    depth: number,
    azimuth: number,
  ): Promise<void> {
    if (visited >= 1024) {
      model.validation.issues.push(
        "Refinement reached 1024 regions. Inspect residual plots and refine measurement calibration.",
      );
      return;
    }
    progress(
      `Solving region ${++visited} · lateral ${t.toFixed(2)} m/s · depth ${depth}`,
      Math.min(0.7, visited / 150),
    );
    await new Promise((r) => setTimeout(r, 0));
    const samples: { s: Scenario; shot: Shot }[] = [];
    for (let i = 0; i < 3; i++)
      for (let j = 0; j < 3; j++) {
        const s = {
          distance: bounds[0] + ((bounds[1] - bounds[0]) * i) / 2,
          radial: bounds[2] + ((bounds[3] - bounds[2]) * j) / 2,
          tangential: t,
          azimuth,
        };
        const shot = optimal(s);
        if (!shot) {
          model.validation.issues.push(
            `No robust shot: distance ${s.distance.toFixed(2)}, radial ${s.radial.toFixed(2)}, lateral ${t.toFixed(2)}. Revisit measurements, geometry or search resolution.`,
          );
          return;
        }
        samples.push({ s, shot });
      }
    const fitted = fitPatch(samples, bounds, t, azimuth);
    let valid = true,
      maxError = 0;
    const checks = [...samples];
    for (const i of [0.25, 0.75])
      for (const j of [0.25, 0.75]) {
        const scenario = {
          distance: bounds[0] + (bounds[1] - bounds[0]) * i,
          radial: bounds[2] + (bounds[3] - bounds[2]) * j,
          tangential: t,
          azimuth,
        };
        const truth = optimal(scenario);
        if (truth) checks.push({ s: scenario, shot: truth });
        else valid = false;
      }
    const residuals = [];
    for (const { s, shot: truth } of checks) {
      const v = evaluatePatch(fitted, s.distance, s.radial);
      model.validation.samples++;
      const angleFraction =
        Math.abs(v.values[0] - truth.angle) /
        Math.max(1e-6, Math.min(...truth.margins.slice(0, 2)));
      const speedFraction =
        Math.abs(v.values[1] - truth.speed) /
        Math.max(1e-6, Math.min(...truth.margins.slice(2)));
      const marginFraction = Math.max(
        ...truth.margins.map(
          (margin, i) =>
            Math.abs(v.values[i + 3] - margin) / Math.max(1e-6, margin),
        ),
      );
      const tofFraction = Math.abs(v.values[2] - truth.tof) / truth.tof;
      maxError = Math.max(
        maxError,
        angleFraction,
        speedFraction,
        marginFraction,
      );
      residuals.push({
        distance: s.distance,
        radial: s.radial,
        tangential: t,
        angleFraction,
        speedFraction,
        marginFraction,
        tofFraction,
      });
      if (
        maxError > 0.1 ||
        tofFraction > 0.01 ||
        v.values.slice(3).some((x) => x <= 0) ||
        !trace(p, s, v.values[0], v.values[1], v.yaw).scored
      )
        valid = false;
    }
    if (!valid && depth < 4) {
      const [a, b, c, d] = bounds,
        mid = (a + b) / 2,
        vr = (c + d) / 2;
      for (const box of [
        [a, mid, c, vr],
        [mid, b, c, vr],
        [a, mid, vr, d],
        [mid, b, vr, d],
      ] as Patch["bounds"][])
        await patch(box, t, depth + 1, azimuth);
    } else {
      model.validation.residuals.push(...residuals);
      model.patches.push(fitted);
      model.validation.maxMarginFraction = Math.max(
        model.validation.maxMarginFraction,
        maxError,
      );
      if (!valid)
        model.validation.issues.push(
          `Quadratic fit failed margin validation in ${bounds.map((x) => x.toFixed(2)).join(", ")}.`,
        );
    }
  }
  // Small patches avoid requiring one polynomial to represent the whole curved feasible region.
  const bands = [-maxMotion, -maxMotion / 2, 0, maxMotion / 2, maxMotion];
  const directions =
    Math.hypot(p.physics.wind[0], p.physics.wind[1]) > 0.001
      ? Array.from({ length: 8 }, (_, i) => (i * Math.PI) / 4)
      : [0];
  for (const azimuth of directions)
    for (const t of bands)
      for (let i = 0; i < 4; i++)
        await patch(
          [
            minD + ((maxD - minD) * i) / 4,
            minD + ((maxD - minD) * (i + 1)) / 4,
            -maxMotion,
            maxMotion,
          ],
          t,
          0,
          azimuth,
        );
  progress(
    "Checking both turrets over the alliance zone and motion envelope…",
    0.8,
  );
  await validateCoverage(p, model, progress);
  model.validation.passed =
    model.validation.issues.length === 0 &&
    model.validation.failures === 0 &&
    model.validation.samples > 0;
  return model;
}
export async function validateCoverage(
  p: Project,
  m: Model,
  progress: (s: string, n: number) => void,
) {
  const g = p.geometry,
    points: CoveragePoint[] = [];
  let done = 0;
  for (let ix = 0; ix <= 16; ix++)
    for (let iy = 0; iy <= 24; iy++) {
      const x = g.zone[0] + ((g.zone[1] - g.zone[0]) * ix) / 16,
        y = g.zone[2] + ((g.zone[3] - g.zone[2]) * iy) / 24;
      // The hub structure is not a robot driving position.
      if (Math.abs(x - g.hub[0]) < 1.05 && Math.abs(y - g.hub[1]) < 1.05)
        continue;
      for (let turret = 0; turret < g.turrets.length; turret++) {
        let passed = true,
          reason = "";
        for (const heading of [0, Math.PI / 2, Math.PI, (3 * Math.PI) / 2])
          for (const direction of [-1, 0, 1, 2, 3, 4, 5, 6, 7])
            for (const omega of [-g.maxOmega, 0, g.maxOmega]) {
              const speed = direction < 0 ? 0 : g.maxSpeed,
                vx = speed * Math.cos((direction * Math.PI) / 4),
                vy = speed * Math.sin((direction * Math.PI) / 4);
              const state = turretState(
                  x,
                  y,
                  heading,
                  vx,
                  vy,
                  omega,
                  g.turrets[turret],
                  p.physics.latency,
                ),
                dx = g.hub[0] - state.x,
                dy = g.hub[1] - state.y,
                d = Math.hypot(dx, dy),
                nx = dx / d,
                ny = dy / d;
              const s = {
                distance: d,
                radial: state.vx * nx + state.vy * ny,
                tangential: -state.vx * ny + state.vy * nx,
                azimuth: Math.atan2(dy, dx),
              };
              try {
                const shot = evaluateModel(m, s);
                m.validation.samples++;
                if (!trace(p, s, shot.angle, shot.speed, shot.yaw).scored)
                  throw Error("Fitted trajectory misses");
                for (const roll of [-g.maxTilt, 0, g.maxTilt])
                  for (const pitch of [-g.maxTilt, 0, g.maxTilt]) {
                    const localYaw =
                      (s.azimuth ?? 0) + shot.yaw - state.heading;
                    const vector = tiltInverse(
                      [
                        shot.speed *
                          Math.cos(rad(shot.angle)) *
                          Math.cos(localYaw),
                        shot.speed *
                          Math.cos(rad(shot.angle)) *
                          Math.sin(localYaw),
                        shot.speed * Math.sin(rad(shot.angle)),
                      ],
                      rad(roll),
                      rad(pitch),
                    );
                    const localAngle = deg(
                        Math.atan2(vector[2], Math.hypot(vector[0], vector[1])),
                      ),
                      hood = mapValue(
                        p.calibration.angle,
                        localAngle - p.physics.angleOffset,
                        true,
                      ),
                      rps = mapValue(
                        conditionalMap(
                          p.calibration.speedByHood,
                          hood,
                          p.calibration.speed,
                        ),
                        shot.speed / p.physics.speedScale,
                        true,
                      );
                    const localAz = Math.atan2(vector[1], vector[0]),
                      rr = rad(roll),
                      pp = rad(pitch);
                    const ax = Math.sin(localAz),
                      ay = -Math.cos(localAz) * Math.cos(rr),
                      az = -Math.cos(localAz) * Math.sin(rr);
                    const wx = Math.cos(pp) * ax + Math.sin(pp) * az,
                      wy = ay,
                      wz = -Math.sin(pp) * ax + Math.cos(pp) * az;
                    const relative = state.heading - (s.azimuth ?? 0),
                      cr = Math.cos(relative),
                      sr = Math.sin(relative);
                    const spinAxis: [number, number, number] = [
                      cr * wx - sr * wy,
                      sr * wx + cr * wy,
                      wz,
                    ];
                    if (
                      !trace(p, s, shot.angle, shot.speed, shot.yaw, spinAxis)
                        .scored
                    )
                      throw Error("Tilted backspin trajectory misses");
                    if (
                      !finite(hood, rps) ||
                      hood < g.hood[0] ||
                      hood > g.hood[1] ||
                      rps < g.rps[0] ||
                      rps > g.rps[1]
                    )
                      throw Error("Tilt correction exceeds mechanism range");
                  }
              } catch (e) {
                passed = false;
                reason = (e as Error).message;
                break;
              }
            }
        points.push({ x, y, turret, passed, reason });
        if (!passed) m.validation.failures++;
      }
      if (++done % 6 === 0) {
        progress(`Coverage ${done}/425 positions`, 0.8 + (0.19 * done) / 425);
        await new Promise((r) => setTimeout(r, 0));
      }
    }
  m.validation.coverage = points;
  if (m.validation.failures)
    m.validation.issues.push(
      `${m.validation.failures} turret/position combinations have unresolved motion or fit coverage.`,
    );
}
export function exportModel(p: Project): Model {
  const m = p.model;
  if (
    !m ||
    !m.validation.passed ||
    !p.calibration.measured ||
    !p.calibration.aeroFitted
  )
    throw Error(
      "Robot export requires measured calibration and passing fit/coverage validation.",
    );
  if (
    JSON.stringify(m.physics) !== JSON.stringify(p.physics) ||
    JSON.stringify(m.geometry) !== JSON.stringify(p.geometry) ||
    JSON.stringify(m.calibration) !== JSON.stringify(p.calibration)
  )
    throw Error("Calibration or physics changed. Regenerate the model.");
  if (
    m.version !== 1 ||
    !m.patches.length ||
    m.validation.samples <= 0 ||
    m.validation.failures !== 0 ||
    m.validation.maxMarginFraction > 0.1 ||
    !m.validation.coverage.length ||
    m.validation.coverage.some((c) => !c.passed)
  )
    throw Error("Incomplete model validation.");
  for (const patch of m.patches) {
    if (
      patch.coefficients.length !== 7 ||
      patch.coefficients.some(
        (c) => c.length !== 6 || !c.every(Number.isFinite),
      ) ||
      patch.yaw.length !== 6 ||
      !patch.yaw.every(Number.isFinite) ||
      patch.scale.some((x) => !Number.isFinite(x) || x <= 0)
    )
      throw Error("Invalid polynomial coefficients.");
    const s = {
      distance: patch.center[0],
      radial: patch.center[1],
      tangential: patch.tangential,
      azimuth: patch.azimuth,
    };
    const q = evaluateModel(m, s);
    if (!trace(p, s, q.angle, q.speed, q.yaw).scored)
      throw Error(
        "Export recheck found a fitted trajectory miss. Regenerate the model.",
      );
  }
  return m;
}
