import {
  type Calibration,
  type MapPoint,
  type Trial,
  type Physics,
  finite,
} from "./types";
import { simulate, physicsAtBearing } from "./physics";
export function mapValue(
  points: MapPoint[],
  x: number,
  inverse = false,
): number {
  if (points.length < 2)
    throw Error("At least two measured calibration points are required");
  const a = points
    .map((p) => (inverse ? { input: p.output, output: p.input } : p))
    .sort((a, b) => a.input - b.input);
  let i = 0;
  while (i < a.length - 2 && x > a[i + 1].input) i++;
  const span = a[i + 1].input - a[i].input;
  if (span <= 1e-9) throw Error("Non-invertible calibration");
  return (
    a[i].output + ((a[i + 1].output - a[i].output) * (x - a[i].input)) / span
  );
}
export function solveLinear(a: number[][], b: number[]): number[] {
  const n = b.length,
    m = a.map((row, i) => [...row, b[i]]);
  for (let k = 0; k < n; k++) {
    let pivot = k;
    for (let j = k + 1; j < n; j++)
      if (Math.abs(m[j][k]) > Math.abs(m[pivot][k])) pivot = j;
    [m[k], m[pivot]] = [m[pivot], m[k]];
    if (Math.abs(m[k][k]) < 1e-12)
      throw Error(
        "Measurements do not independently identify these parameters",
      );
    const d = m[k][k];
    for (let j = k; j <= n; j++) m[k][j] /= d;
    for (let i = 0; i < n; i++)
      if (i !== k) {
        const f = m[i][k];
        for (let j = k; j <= n; j++) m[i][j] -= f * m[k][j];
      }
  }
  return m.map((row) => row[n]);
}
export function leastSquares(x: number[][], y: number[]): number[] {
  const n = x[0].length,
    ata = Array.from({ length: n }, () => Array(n).fill(0)),
    atb = Array(n).fill(0);
  x.forEach((row, i) =>
    row.forEach((v, j) => {
      atb[j] += v * y[i];
      row.forEach((w, k) => (ata[j][k] += v * w));
    }),
  );
  return solveLinear(ata, atb);
}
function grouped(
  rows: Trial[],
  input: "hood" | "rps",
  output: "elevation" | "speed" | "spin",
) {
  const g = new Map<number, number[]>(),
    inputs = new Map<number, number[]>();
  for (const r of rows)
    if (r[output] !== undefined && finite(r[input], r[output]!)) {
      const key =
        input === "rps" ? (r.commandRps ?? Math.round(r.rps)) : r[input];
      g.set(key, [...(g.get(key) ?? []), r[output]!]);
      inputs.set(key, [...(inputs.get(key) ?? []), r[input]]);
    }
  let sum = 0,
    n = 0;
  const points = [...g]
    .sort((a, b) => a[0] - b[0])
    .map(([input, values]) => {
      const output = values.reduce((a, b) => a + b, 0) / values.length;
      values.forEach((v) => {
        sum += (v - output) ** 2;
        n++;
      });
      return {
        input:
          inputs.get(input)!.reduce((a, b) => a + b, 0) /
          inputs.get(input)!.length,
        output,
      };
    });
  points.sort((a, b) => a.input - b.input);
  return {
    points,
    sigma: Math.sqrt(sum / Math.max(1, n - points.length)),
    repeats: n - points.length,
  };
}
export function conditionalMap(
  groups: { hood: number; points: MapPoint[] }[] | undefined,
  hood: number,
  fallback: MapPoint[],
): MapPoint[] {
  if (!groups?.length) return fallback;
  if (groups.length === 1) return groups[0].points;
  const ordered = [...groups].sort((a, b) => a.hood - b.hood);
  let i = 0;
  while (i < ordered.length - 2 && hood > ordered[i + 1].hood) i++;
  const a = ordered[i],
    b = ordered[i + 1],
    t = (hood - a.hood) / (b.hood - a.hood);
  return [...new Set([...a.points, ...b.points].map((p) => p.input))]
    .sort((a, b) => a - b)
    .map((input) => ({
      input,
      output:
        mapValue(a.points, input) +
        (mapValue(b.points, input) - mapValue(a.points, input)) * t,
    }));
}
export function calibrate(rows: Trial[]): Calibration {
  const a = grouped(rows, "hood", "elevation"),
    v = grouped(rows, "rps", "speed"),
    s = grouped(rows, "rps", "spin"),
    warnings: string[] = [];
  const monotonic = (p: MapPoint[]) =>
    p.length >= 2 &&
    p.every(
      (x, i) =>
        i === 0 ||
        (x.output - p[i - 1].output) * (p[1].output - p[0].output) > 0,
    );
  if (!monotonic(a.points))
    warnings.push(
      "Measure a strictly monotonic hood-command → release-elevation mapping.",
    );
  if (!monotonic(v.points))
    warnings.push("Measure a strictly increasing RPS → exit-speed mapping.");
  if (v.points.length >= 2 && v.points[1].output < v.points[0].output)
    warnings.push("Exit speed must increase with RPS.");
  if (s.points.length < 2)
    warnings.push("Measure backspin at two or more wheel speeds.");
  if (a.repeats < 2 || v.repeats < 2 || a.sigma === 0 || v.sigma === 0)
    warnings.push(
      "Repeat release-angle and speed measurements to estimate nonzero variability.",
    );
  const hoodGroups = [...new Set(rows.map((r) => r.hood))].sort(
    (a, b) => a - b,
  );
  const speedByHood = hoodGroups
    .map((hood) => ({
      hood,
      points: grouped(
        rows.filter((r) => r.hood === hood),
        "rps",
        "speed",
      ).points,
    }))
    .filter((g) => g.points.length >= 2);
  const spinByHood = hoodGroups
    .map((hood) => ({
      hood,
      points: grouped(
        rows.filter((r) => r.hood === hood),
        "rps",
        "spin",
      ).points,
    }))
    .filter((g) => g.points.length >= 2);
  const speedRows = rows.filter(
    (r) => r.speed !== undefined && Number.isFinite(r.speed),
  );
  const degreesOfFreedom =
    speedRows.length -
    (speedByHood.length
      ? speedByHood.reduce((sum, g) => sum + g.points.length, 0)
      : v.points.length);
  const sigmaSpeed =
    speedRows.length && degreesOfFreedom > 0 && v.points.length >= 2
      ? Math.sqrt(
          speedRows.reduce(
            (sum, r) =>
              sum +
              (r.speed! -
                mapValue(
                  conditionalMap(speedByHood, r.hood, v.points),
                  r.rps,
                )) **
                2,
            0,
          ) / degreesOfFreedom,
        )
      : 0;
  if (!(sigmaSpeed > 0) && !warnings.some((w) => w.startsWith("Repeat")))
    warnings.push(
      "Repeat speed measurements at each hood/command setting to estimate variability.",
    );
  for (const group of speedByHood)
    if (
      !monotonic(group.points) ||
      group.points[1].output < group.points[0].output
    )
      warnings.push(
        `Non-increasing speed transfer at hood ${group.hood}. Recheck measurements.`,
      );
  return {
    speedByHood,
    spinByHood,
    measured: warnings.length === 0,
    angle: a.points,
    speed: v.points,
    spin: s.points,
    sigmaAngle: a.sigma,
    sigmaSpeed,
    warnings,
    diagnostics: [
      {
        parameter: "Release angle repeatability (deg)",
        value: a.sigma,
        uncertainty: a.repeats > 1 ? a.sigma / Math.sqrt(2 * a.repeats) : null,
        source: "Measured within-command residuals",
      },
      {
        parameter: "Exit speed repeatability (m/s)",
        value: sigmaSpeed,
        uncertainty:
          degreesOfFreedom > 1
            ? sigmaSpeed / Math.sqrt(2 * degreesOfFreedom)
            : null,
        source: "Measured within-command residuals",
      },
    ],
    aeroFitted: false,
  };
}
export function fitAerodynamics(rows: Trial[], physics: Physics) {
  const usable = rows.filter(
    (r) =>
      finite(
        r.elevation ?? NaN,
        r.speed ?? NaN,
        r.spin ?? NaN,
        r.tof ?? NaN,
        r.landingX ?? NaN,
        r.landingY ?? NaN,
      ) && (r.tof ?? 0) > 0,
  );
  if (usable.length < 6)
    throw Error(
      "Need at least six measured flights with elevation, speed, spin, time and crossing/landing X/Y.",
    );
  const keys = ["drag", "magnus", "spinDecay"] as const,
    p = { ...physics };
  const residual = (q: Physics) =>
    usable.flatMap((r) => {
      const f = simulate(
        physicsAtBearing(q, r.azimuth ?? 0),
        {
          distance: r.distance ?? 0,
          radial: r.radial ?? 0,
          tangential: r.tangential ?? 0,
        },
        r.elevation!,
        r.speed!,
        r.spin!,
        r.yaw ?? 0,
        r.tof,
      );
      return [
        f.final[0] - r.landingX!,
        f.final[1] - r.landingY!,
        f.final[2] - (r.outcome === "ground" ? q.radius : q.hubHeight),
      ];
    });
  let err = residual(p),
    damping = 0.001;
  for (let iter = 0; iter < 35; iter++) {
    const jac = keys.map((k) => {
      const step = 0.0001,
        q = { ...p, [k]: p[k] + step };
      const r = residual(q);
      return r.map((x, i) => (x - err[i]) / step);
    });
    const a = keys.map((_, j) =>
        keys.map((_, k) => jac[j].reduce((s, x, i) => s + x * jac[k][i], 0)),
      ),
      b = keys.map((_, j) => -jac[j].reduce((s, x, i) => s + x * err[i], 0));
    const step = solveLinear(
      a.map((r, i) => r.map((x, j) => x + (i === j ? damping : 0))),
      b,
    );
    const q = { ...p };
    keys.forEach(
      (k, i) =>
        (q[k] = Math.max(
          0,
          Math.min(k === "spinDecay" ? 10 : 3, p[k] + step[i]),
        )),
    );
    const next = residual(q);
    if (
      next.reduce((s, x) => s + x * x, 0) < err.reduce((s, x) => s + x * x, 0)
    ) {
      Object.assign(p, q);
      err = next;
      damping /= 2;
    } else damping *= 4;
  }
  const jac = keys.map((k) => {
    const q = { ...p, [k]: p[k] + 0.0001 };
    return residual(q).map((x, i) => (x - err[i]) / 0.0001);
  });
  const gram = keys.map((_, j) =>
    keys.map((_, k) => jac[j].reduce((s, x, i) => s + x * jac[k][i], 0)),
  );
  const variance =
    err.reduce((s, x) => s + x * x, 0) / Math.max(1, err.length - 3);
  const diagnostics = keys.map((k, i) => {
    let uncertainty: number | null = null;
    try {
      uncertainty = Math.sqrt(
        Math.max(
          0,
          solveLinear(
            gram,
            keys.map((_, j) => (i === j ? 1 : 0)),
          )[i],
        ) * variance,
      );
    } catch {
      /* report nonidentifiability */
    }
    return {
      parameter: k,
      value: p[k],
      uncertainty,
      source: "Measured flight residual fit",
    };
  });
  if (
    diagnostics.some(
      (d) => d.uncertainty === null || !Number.isFinite(d.uncertainty),
    )
  )
    throw Error(
      "Aerodynamic parameters are not independently identifiable. Include varied spin, speeds and flight times.",
    );
  return { physics: p, diagnostics, rmse: Math.sqrt(variance) };
}
