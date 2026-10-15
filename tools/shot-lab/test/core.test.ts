import { test } from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import { defaults, type Model, type Trial, rad } from "../src/types";
import {
  simulate,
  acceleration,
  tiltInverse,
  turretState,
} from "../src/physics";
import { mapValue, calibrate, fitAerodynamics } from "../src/calibration";
import { evaluateModel, exportModel, solveShot, trace } from "../src/solver";
import { parseProject, save, load } from "../src/storage";
import { csvRows, csvEncode, readLog, snapshot, valueAt } from "../src/logs";
const close = (a: number, b: number, t = 1e-7) =>
  assert.ok(Math.abs(a - b) < t, `${a} != ${b}`);
test("RK4 matches vacuum projectile trajectory and crossing time", () => {
  const p = { ...defaults().physics, drag: 0, magnus: 0 };
  const f = simulate(
    p,
    { distance: 4, radial: 0, tangential: 0 },
    60,
    8,
    0,
    0,
    1,
  );
  close(f.final[0], 4);
  close(f.final[2], p.launchHeight + 8 * Math.sin(rad(60)) - 0.5 * 9.81);
});
test("backspin lifts; topspin lowers; drag opposes relative air velocity", () => {
  const p = { ...defaults().physics, magnus: 0.2 };
  const a = acceleration([8, 0, 2], [0, -50, 0], p),
    b = acceleration([8, 0, 2], [0, 50, 0], p);
  assert.ok(a[2] > b[2]);
  assert.ok(acceleration([8, 0, 0], [0, 0, 0], p)[0] < 0);
  const still = { ...p, wind: [8, 0, 0] as [number, number, number] };
  close(acceleration([8, 0, 0], [0, 0, 0], still)[0], 0);
});
test("whole ball clears hub; rim contact is excluded", () => {
  const p = { ...defaults().physics, drag: 0, magnus: 0 },
    s = { distance: 4, radial: 0, tangential: 0 };
  const base = simulate(p, s, 65, 8.5, 0);
  assert.ok(base.crossing);
  const center = base.crossing![0];
  assert.equal(
    simulate(p, { ...s, distance: center }, 65, 8.5, 0).scored,
    true,
  );
  const rim = simulate(
    p,
    { ...s, distance: center + p.hubRadius - 0.02 },
    65,
    8.5,
    0,
  );
  assert.equal(rim.scored, false);
  assert.equal(rim.collision, true);
});
test("calibration supports decreasing hood elevation; inverse round trips", () => {
  const points = [
    { input: 13, output: 80 },
    { input: 29.85, output: 40 },
  ];
  for (const x of [13, 20, 29.85])
    close(mapValue(points, mapValue(points, x), true), x);
});
test("measurements require repeatability and never borrow the old shot table", () => {
  assert.equal(calibrate([]).measured, false);
  const rows: Trial[] = [];
  for (const hood of [13, 29.85])
    for (const rps of [30, 80])
      for (const error of [-0.2, 0, 0.2])
        rows.push({
          id: String(rows.length),
          turret: "Left",
          hood,
          rps,
          elevation: 90 - hood + error,
          speed: rps * 0.15 + error * 0.2,
          spin: rps * 0.3,
        });
  assert.equal(calibrate(rows).measured, true);
  assert.ok(calibrate(rows).sigmaAngle > 0);
});
test("aerodynamic fit rejects unidentifiable data and missing observations", () => {
  assert.throws(() => fitAerodynamics([], defaults().physics), /six/);
  const rows = Array.from({ length: 6 }, (_, i) => ({
    id: String(i),
    turret: "Left",
    hood: 20,
    rps: 50,
    elevation: 60,
    speed: 8,
    spin: 0,
    tof: 1,
    landingX: 4,
    landingY: 0,
  }));
  assert.throws(
    () => fitAerodynamics(rows, defaults().physics),
    /identifiable/,
  );
});
test("TS evaluator matches archived coefficient fixture including lateral interpolation", () => {
  const m = JSON.parse(
    fs.readFileSync(new URL("./fixtures/model.json", import.meta.url), "utf8"),
  ) as Model;
  const x = JSON.parse(
    fs.readFileSync(
      new URL("./fixtures/evaluation.json", import.meta.url),
      "utf8",
    ),
  );
  const shot = evaluateModel(m, x.scenario);
  [shot.angle, shot.speed, shot.tof, ...shot.margins].forEach((v, i) =>
    close(v, x.values[i]),
  );
  close(shot.yaw, x.yaw);
  assert.throws(
    () => evaluateModel(m, { distance: 2, radial: 1, tangential: 5 }),
    /coverage/,
  );
  const broken = structuredClone(m);
  broken.patches[0].coefficients[0][0] = NaN;
  assert.throws(
    () => evaluateModel(broken, { distance: 2, radial: 1, tangential: 0 }),
    /invalid/,
  );
});
test("turret rotation velocity, release latency and tilt inverse preserve conventions", () => {
  const a = turretState(1, 2, 0, 1, 0, 2, [-0.2, 0.2], 0.05),
    b = turretState(1, 2, 0, 1, 0, 2, [-0.2, -0.2], 0.05);
  assert.notEqual(a.vx, b.vx);
  close(a.heading, 0.1);
  const v = tiltInverse([3, 0, 4], 0, 0.1);
  close(Math.hypot(...v), 5);
  assert.ok(v[2] > 4);
});
test("project persistence round trips and corrupt projects fail explicitly", () => {
  let value: string | null = null;
  const memory = {
    getItem: () => value,
    setItem: (_k: string, v: string) => (value = v),
  };
  const p = defaults();
  save(memory, p);
  assert.deepEqual(load(memory), p);
  assert.throws(() => parseProject("{}"), /incomplete/);
  p.physics.dt = 0;
  assert.throws(() => parseProject(JSON.stringify(p)), /Invalid/);
  assert.throws(() => exportModel(defaults()), /requires/);
});
test("CSV preserves commas, quoted notes and blank measurements", () => {
  const text = csvEncode(
    [{ hood: 13, rps: 40, notes: 'one, "two"' }],
    ["hood", "rps", "notes"],
  );
  assert.equal(csvRows(text)[0].notes, 'one, "two"');
  assert.throws(() => csvRows('a\n"bad'), /Unterminated/);
});
test("binary WPILOG validates header, timestamp alignment and missing channels", () => {
  const file = fs.readFileSync(
    new URL("./fixtures/telemetry.wpilog", import.meta.url),
  );
  const log = readLog(
    file.buffer.slice(file.byteOffset, file.byteOffset + file.byteLength),
  );
  assert.ok(log.channels.length >= 3);
  const measured = log.channels.find((c) =>
    c.name.endsWith("MeasuredWheelRps"),
  )!;
  assert.equal(valueAt(measured, 1.01), 48.5);
  assert.equal(valueAt(measured, 0.9), undefined);
  assert.equal(valueAt(undefined, 1), undefined);
  assert.ok(Object.keys(snapshot(log, 1.01)).length >= 3);
  assert.throws(() => snapshot(log, 100), /timestamp/);
  assert.throws(() => readLog(new ArrayBuffer(2)), /Truncated/);
  assert.throws(
    () =>
      readLog(
        file.buffer.slice(
          file.byteOffset,
          file.byteOffset + file.byteLength - 1,
        ),
      ),
    /Truncated/,
  );
});
test("stationary optimizer finds an actual scoring region and asymmetric margins", () => {
  const p = defaults();
  p.physics.drag = 0;
  p.calibration = {
    ...calibrate([]),
    measured: true,
    sigmaAngle: 0.2,
    sigmaSpeed: 0.05,
    angle: [
      { input: 13, output: 80 },
      { input: 29.85, output: 40 },
    ],
    speed: [
      { input: 0, output: 0 },
      { input: 100, output: 20 },
    ],
    spin: [
      { input: 0, output: 0 },
      { input: 100, output: 0 },
    ],
  };
  const s = { distance: 3, radial: 0, tangential: 0 },
    shot = solveShot(p, s);
  assert.ok(shot);
  assert.ok(shot.margins.every((x) => x > 0));
  assert.ok(trace(p, s, shot.angle, shot.speed, shot.yaw).scored);
});
test("hood-dependent compression transfer is invertible at intermediate hood settings", async () => {
  const { conditionalMap } = await import("../src/calibration");
  const groups = [
    {
      hood: 13,
      points: [
        { input: 20, output: 3 },
        { input: 80, output: 12 },
      ],
    },
    {
      hood: 29,
      points: [
        { input: 20, output: 4 },
        { input: 80, output: 16 },
      ],
    },
  ];
  const points = conditionalMap(groups, 21, []);
  close(mapValue(points, 50), 8.75);
  close(mapValue(points, 8.75, true), 50);
});
test("aerodynamic fit recovers known coefficients from varied synthetic flights", () => {
  const truth = {
    ...defaults().physics,
    drag: 0.35,
    magnus: 0.08,
    spinDecay: 0.7,
  };
  const rows: Trial[] = [];
  for (const angle of [48, 65])
    for (const speed of [7, 11])
      for (const spin of [5, 20])
        for (const tof of [0.5, 1]) {
          const f = simulate(
            truth,
            { distance: 0, radial: 0, tangential: 0 },
            angle,
            speed,
            spin,
            0,
            tof,
          );
          // Fit API uses ground or hub-height positions; choose observed time at a known plane via per-trial release geometry below.
          rows.push({
            id: String(rows.length),
            turret: "Left",
            hood: 20,
            rps: 50,
            elevation: angle,
            speed,
            spin,
            tof,
            landingX: f.final[0],
            landingY: f.final[1],
            notes: String(f.final[2]),
          });
        }
  // Use hub-plane crossing observations rather than arbitrary times.
  const crossings: Trial[] = [];
  for (const angle of [48, 60, 72])
    for (const speed of [8, 10, 12])
      for (const spin of [5, 20]) {
        const f = simulate(
          truth,
          { distance: 100, radial: 0, tangential: 0 },
          angle,
          speed,
          spin,
        );
        if (f.crossing)
          crossings.push({
            id: String(crossings.length),
            turret: "Left",
            hood: 20,
            rps: 50,
            elevation: angle,
            speed,
            spin,
            tof: f.tof,
            landingX: f.crossing[0],
            landingY: f.crossing[1],
          });
      }
  const fitted = fitAerodynamics(crossings, {
    ...truth,
    drag: 0.47,
    magnus: 0.03,
    spinDecay: 0.2,
  });
  close(fitted.physics.drag, truth.drag, 0.02);
  close(fitted.physics.magnus, truth.magnus, 0.02);
  close(fitted.physics.spinDecay, truth.spinDecay, 0.12);
});
test("moving-shot solve re-simulates correctly for both lateral directions", () => {
  const p = defaults();
  p.physics.drag = 0.2;
  p.physics.magnus = 0.05;
  p.calibration = {
    ...calibrate([]),
    measured: true,
    sigmaAngle: 0.2,
    sigmaSpeed: 0.05,
    angle: [
      { input: 13, output: 80 },
      { input: 29.85, output: 40 },
    ],
    speed: [
      { input: 0, output: 0 },
      { input: 100, output: 20 },
    ],
    spin: [
      { input: 0, output: 0 },
      { input: 100, output: 20 },
    ],
  };
  const left = { distance: 3, radial: 1, tangential: 2.5 },
    right = { ...left, tangential: -2.5 };
  const a = solveShot(p, left),
    b = solveShot(p, right);
  assert.ok(a && b);
  assert.ok(trace(p, left, a.angle, a.speed, a.yaw).scored);
  assert.ok(trace(p, right, b.angle, b.speed, b.yaw).scored);
  close(a.yaw, -b.yaw, 1e-6);
});
test("partial repeated measurements return diagnostics instead of throwing", () => {
  const rows = Array.from({ length: 3 }, (_, i) => ({
    id: String(i),
    turret: "Left",
    hood: 13,
    rps: 40,
    elevation: 70 + i * 0.1,
    speed: 8 + i * 0.01,
    spin: 10,
  }));
  const result = calibrate(rows);
  assert.equal(result.measured, false);
  assert.ok(result.warnings.length > 0);
});
test("fast search integrator agrees with displayed flight on clean trajectories", async () => {
  const { scoringFlight } = await import("../src/physics");
  const p = { ...defaults().physics, drag: 0.35, magnus: 0.05 };
  const s = { distance: 3, radial: 0.5, tangential: 1 };
  const a = simulate(p, s, 60, 9, 15, -0.15),
    b = scoringFlight(p, s, 60, 9, 15, -0.15);
  assert.ok(!a.collision);
  close(a.tof, b.tof, 1e-8);
  a.final.forEach((x, i) => close(x, b.final[i], 1e-8));
  assert.equal(a.scored, b.scored);
});
