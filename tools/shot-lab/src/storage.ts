import { defaults, type Project } from "./types";
export const STORAGE_KEY = "chuck-shot-lab-v1";
export function parseProject(text: string): Project {
  const p = JSON.parse(text) as Project;
  if (
    !p ||
    p.version !== 1 ||
    !p.physics ||
    !p.geometry ||
    !Array.isArray(p.trials) ||
    !p.calibration
  )
    throw Error("Unsupported or incomplete Shot Lab project");
  const vector = (a: unknown, n: number) =>
    Array.isArray(a) &&
    a.length === n &&
    a.every((x) => typeof x === "number" && Number.isFinite(x));
  if (
    !vector(p.physics.wind, 3) ||
    !vector(p.geometry.zone, 4) ||
    !vector(p.geometry.hub, 2) ||
    !vector(p.geometry.hood, 2) ||
    !vector(p.geometry.rps, 2) ||
    !Array.isArray(p.geometry.turrets) ||
    p.geometry.turrets.length !== 2 ||
    p.geometry.turrets.some((t) => !vector(t, 2))
  )
    throw Error("Incomplete geometry or wind vector");
  for (const k of ["maxSpeed", "maxOmega", "maxTilt"] as const)
    if (!Number.isFinite(p.geometry[k])) throw Error("Invalid motion bounds");
  for (const k of [
    "angle",
    "speed",
    "spin",
    "warnings",
    "diagnostics",
  ] as const)
    if (!Array.isArray(p.calibration[k])) throw Error("Incomplete calibration");
  if (!Array.isArray(p.comparison)) p.comparison = [];
  if (
    p.model &&
    (!p.model.validation ||
      !Array.isArray(p.model.patches) ||
      !Array.isArray(p.model.validation.issues) ||
      !Array.isArray(p.model.validation.coverage))
  )
    throw Error("Incomplete fitted model");
  for (const trial of p.trials)
    if (
      typeof trial.id !== "string" ||
      !Number.isFinite(trial.hood) ||
      !Number.isFinite(trial.rps)
    )
      throw Error("Invalid measurement trial");
  for (const k of ["dt", "mass", "radius", "hubRadius", "speedScale"] as const)
    if (!Number.isFinite(p.physics[k]) || p.physics[k] <= 0)
      throw Error(`Invalid physics ${k}`);
  for (const k of [
    "gravity",
    "density",
    "drag",
    "magnus",
    "spinDecay",
    "latency",
    "spinScale",
    "rimRadius",
  ] as const)
    if (!Number.isFinite(p.physics[k]) || p.physics[k] < 0)
      throw Error(`Invalid physics ${k}`);
  if (
    p.physics.radius + p.physics.rimRadius >= p.physics.hubRadius ||
    p.physics.restitution < 0 ||
    p.physics.restitution > 1
  )
    throw Error("Invalid ball/rim geometry or restitution");
  if (
    p.geometry.zone[0] >= p.geometry.zone[1] ||
    p.geometry.zone[2] >= p.geometry.zone[3] ||
    p.geometry.maxSpeed < 0 ||
    p.geometry.maxOmega < 0 ||
    p.geometry.maxTilt < 0
  )
    throw Error("Invalid field or motion bounds");
  if (p.physics.dt < 0.001 || p.physics.dt > 0.05)
    throw Error("Integration step must be 0.001–0.05 seconds");
  const numbers = (x: unknown): boolean =>
    typeof x === "number"
      ? Number.isFinite(x)
      : Array.isArray(x)
        ? x.every(numbers)
        : x !== null && typeof x === "object"
          ? Object.values(x).every(numbers)
          : true;
  if (!numbers(p)) throw Error("Project contains non-finite numbers");
  if (
    p.geometry.hood[0] >= p.geometry.hood[1] ||
    p.geometry.rps[0] >= p.geometry.rps[1]
  )
    throw Error("Invalid mechanism bounds");
  return p;
}
export function load(storage: Pick<Storage, "getItem">): Project {
  const text = storage.getItem(STORAGE_KEY);
  return text ? parseProject(text) : defaults();
}
export function save(storage: Pick<Storage, "setItem">, p: Project) {
  storage.setItem(STORAGE_KEY, JSON.stringify(p));
}
