import { test } from "node:test";
import assert from "node:assert/strict";
import {
  baseline,
  sweep,
  compensate,
  flight,
  evaluate,
  generate,
  type RadialMap,
} from "../src/precomputed";
import { readFileSync, existsSync, writeFileSync, mkdirSync } from "node:fs";
test("enumerated scoring bands have lower/upper rim-safe limits; robust winner scores", () => {
  const { best, bands } = sweep(baseline, 3.5, 0);
  assert.ok(best);
  assert.ok(bands.length > 5);
  for (const b of bands) {
    assert.ok(b.lower < b.upper);
    assert.ok(
      flight(baseline, 3.5, 0, b.angle, (b.lower + b.upper) / 2).scored,
    );
  }
  assert.ok(flight(baseline, 3.5, 0, best.values[0], best.values[1]).scored);
  assert.ok(best.values.slice(3).every((x) => x > 0));
});
test("3D tangential subtraction preserves radial and vertical components and changes elevation/speed", () => {
  for (const t of [-3, -1, 0, 2, 4]) {
    const q = compensate(72, 10, t);
    assert.ok(
      Math.abs(q.vector[0] - 10 * Math.cos((72 * Math.PI) / 180)) < 1e-10,
    );
    assert.equal(q.vector[1] + t, 0);
    assert.ok(
      Math.abs(q.vector[2] - 10 * Math.sin((72 * Math.PI) / 180)) < 1e-10,
    );
    if (t) {
      assert.ok(q.speed > 10);
      assert.ok(q.angle < 72);
    }
    assert.ok(flight(baseline, 3.5, 0, 75, 8.75, t).scored);
  }
});
test("radial sign changes the required launch speed", () => {
  const a = sweep(baseline, 3.5, -1).best!,
    b = sweep(baseline, 3.5, 1).best!;
  assert.ok(a.values[1] > b.values[1]);
});
test("normalized quadratic evaluation includes mixed and squared terms; no uncovered fallback", () => {
  const m = {
    patches: [
      {
        bounds: [2, 4, -1, 1],
        center: [3, 0],
        scale: [1, 1],
        coefficients: Array.from({ length: 7 }, () => [1, 2, 3, 4, 5, 6]),
      },
    ],
  } as RadialMap;
  assert.deepEqual(
    evaluate(m, 3.5, 0.25),
    Array(7).fill(1 + 1 + 0.75 + 1 + 0.625 + 0.375),
  );
  assert.throws(() => evaluate(m, 5, 0), /Unvalidated/);
});
test("long map generation can cancel without producing a partial valid model", async () => {
  await assert.rejects(
    () =>
      generate(
        baseline,
        [3, 4, -1, 1],
        () => {},
        () => true,
      ),
    /Cancelled/,
  );
});
test("baseline map has validated radial patches and shared Java evaluation fixtures", () => {
  const path = "../../src/main/deploy/chuck_shot_model.json";
  assert.ok(existsSync(path), "Packaged baseline map is required");
  const m = JSON.parse(readFileSync(path, "utf8")) as RadialMap;
  assert.equal(m.version, 2);
  assert.ok(m.patches.length > 0);
  assert.equal(m.validation.physicalValidation, false);
  assert.ok(m.validation.maxErrorFraction < 0.1);
  const fixtures = m.patches
    .filter((_, i) => i % Math.max(1, Math.floor(m.patches.length / 30)) === 0)
    .map((p) => {
      const d = p.center[0] + 0.37 * p.scale[0],
        r = p.center[1] - 0.23 * p.scale[1],
        values = evaluate(m, d, r);
      assert.ok(flight(m.config, d, r, values[0], values[1]).scored);
      return { distance: d, radial: r, values };
    });
  mkdirSync("test/fixtures", { recursive: true });
  writeFileSync(
    "test/fixtures/radial-v2-evaluation.json",
    JSON.stringify(fixtures),
  );
});
