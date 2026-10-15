import { evaluate, compensate, flight, type RadialMap } from "./precomputed";
import { defaults } from "./types";
export function validateAlliance(m: RadialMap) {
  const g = defaults().geometry;
  let total = 0,
    passed = 0,
    uncovered = 0,
    mechanism = 0,
    misses = 0;
  const points: { x: number; y: number; tested: number; passed: number }[] = [];
  for (let ix = 0; ix <= 12; ix++)
    for (let iy = 0; iy <= 18; iy++) {
      const x = g.zone[0] + ((g.zone[1] - g.zone[0]) * ix) / 12,
        y = g.zone[2] + ((g.zone[3] - g.zone[2]) * iy) / 18;
      if (Math.abs(x - g.hub[0]) < 1.05 && Math.abs(y - g.hub[1]) < 1.05)
        continue;
      let checked = 0,
        ok = 0;
      for (const o of g.turrets)
        for (const heading of [0, Math.PI / 2, Math.PI, (3 * Math.PI) / 2]) {
          const ox = o[0] * Math.cos(heading) - o[1] * Math.sin(heading),
            oy = o[0] * Math.sin(heading) + o[1] * Math.cos(heading),
            dx = g.hub[0] - x - ox,
            dy = g.hub[1] - y - oy,
            d = Math.hypot(dx, dy);
          for (const speed of [0, 1.25, 2.5])
            for (
              let direction = 0;
              direction < (speed === 0 ? 1 : 8);
              direction++
            )
              for (const omega of [-g.maxOmega, 0, g.maxOmega]) {
                const vx =
                    speed * Math.cos((direction * Math.PI) / 4) - omega * oy,
                  vy = speed * Math.sin((direction * Math.PI) / 4) + omega * ox,
                  r = (vx * dx + vy * dy) / d,
                  t = (-vx * dy + vy * dx) / d;
                total++;
                checked++;
                try {
                  const q = evaluate(m, d, r),
                    c = compensate(q[0], q[1], t);
                  if (
                    Math.hypot(r, t) > 4.25 ||
                    c.angle < Math.min(m.config.angleMin, m.config.angleMax) ||
                    c.angle > Math.max(m.config.angleMin, m.config.angleMax) ||
                    c.speed >
                      100 * 2 * Math.PI * m.config.radius * m.config.transfer
                  ) {
                    mechanism++;
                    continue;
                  }
                  if (!flight(m.config, d, r, q[0], q[1], t).scored) {
                    misses++;
                    continue;
                  }
                  passed++;
                  ok++;
                } catch {
                  uncovered++;
                }
              }
        }
      points.push({ x, y, tested: checked, passed: ok });
    }
  const report = {
    modelVersion: 2,
    physicalValidation: false as const,
    description:
      "Uncalibrated baseline; flat chassis, zero release delay and neutral trims. Two turret offsets, four headings, stationary/1.25/2.5 m/s, eight directions, zero/positive/negative max chassis rotation.",
    total,
    passed,
    uncovered,
    mechanismLimitFailures: mechanism,
    simulatedMisses: misses,
    points,
  };
  return report;
}
