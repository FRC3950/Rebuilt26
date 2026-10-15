export type Vec = [number, number, number];
export interface Physics {
  gravity: number;
  density: number;
  drag: number;
  magnus: number;
  spinDecay: number;
  mass: number;
  radius: number;
  wind: Vec;
  launchHeight: number;
  hubHeight: number;
  hubRadius: number;
  rimRadius: number;
  restitution: number;
  dt: number;
  latency: number;
  speedScale: number;
  spinScale: number;
  angleOffset: number;
}
export interface Geometry {
  hub: [number, number];
  zone: [number, number, number, number];
  turrets: [number, number][];
  hood: [number, number];
  rps: [number, number];
  maxSpeed: number;
  maxOmega: number;
  maxTilt: number;
}
export interface Trial {
  id: string;
  turret: string;
  timestamp?: number;
  hood: number;
  rps: number;
  commandRps?: number;
  elevation?: number;
  speed?: number;
  spin?: number;
  tof?: number;
  distance?: number;
  radial?: number;
  tangential?: number;
  yaw?: number;
  azimuth?: number;
  landingX?: number;
  landingY?: number;
  outcome?: string;
  notes?: string;
  source?: string;
  telemetry?: Record<string, number | string | boolean | number[]>;
}
export interface MapPoint {
  input: number;
  output: number;
}
export interface Calibration {
  speedByHood?: { hood: number; points: MapPoint[] }[];
  spinByHood?: { hood: number; points: MapPoint[] }[];
  measured: boolean;
  angle: MapPoint[];
  speed: MapPoint[];
  spin: MapPoint[];
  sigmaAngle: number;
  sigmaSpeed: number;
  warnings: string[];
  diagnostics: {
    parameter: string;
    value: number;
    uncertainty: number | null;
    source: string;
  }[];
  aeroFitted: boolean;
}
export interface Scenario {
  distance: number;
  radial: number;
  tangential: number;
  azimuth?: number;
}
export interface Shot {
  angle: number;
  speed: number;
  tof: number;
  yaw: number;
  margins: number[];
  score: number;
}
export interface Patch {
  bounds: [number, number, number, number];
  center: [number, number];
  scale: [number, number];
  coefficients: number[][];
  yaw: number[];
  tangential: number;
  azimuth: number;
}
export interface CoveragePoint {
  x: number;
  y: number;
  turret: number;
  passed: boolean;
  reason: string;
}
export interface Validation {
  passed: boolean;
  samples: number;
  failures: number;
  maxMarginFraction: number;
  residuals: {
    distance: number;
    radial: number;
    tangential: number;
    angleFraction: number;
    speedFraction: number;
    marginFraction: number;
    tofFraction: number;
  }[];
  physicalValidated: boolean;
  coverage: CoveragePoint[];
  issues: string[];
}
export interface Model {
  version: 1;
  name: string;
  createdAt: string;
  units: string;
  calibration: Calibration;
  physics: Physics;
  geometry: Geometry;
  patches: Patch[];
  validation: Validation;
}
export interface Project {
  version: 1;
  name: string;
  physics: Physics;
  geometry: Geometry;
  trials: Trial[];
  calibration: Calibration;
  model?: Model;
  comparison: { d: number; hoodDeg: number; rps: number; tof: number }[];
}
export const defaults = (): Project => ({
  version: 1,
  name: "Chuck Norris",
  physics: {
    gravity: 9.81,
    density: 1.2041,
    drag: 0.47,
    magnus: 0,
    spinDecay: 0,
    mass: 0.20335424,
    radius: 0.075,
    wind: [0, 0, 0],
    launchHeight: 0.445,
    hubHeight: 1.83,
    hubRadius: 0.56,
    rimRadius: 0.025,
    restitution: 0.2,
    dt: 0.01,
    latency: 0.05,
    speedScale: 1,
    spinScale: 1,
    angleOffset: 0,
  },
  geometry: {
    hub: [4.6256194, 4.0346376],
    zone: [0, 5.1816, 0, 8.0756125],
    turrets: [
      [-0.18415, 0.19685],
      [-0.18415, -0.19685],
    ],
    hood: [13, 29.85],
    rps: [0, 100],
    maxSpeed: 2.5,
    maxOmega: 6.254342289681399,
    maxTilt: 5,
  },
  trials: [],
  calibration: {
    measured: false,
    angle: [],
    speed: [],
    spin: [],
    sigmaAngle: 0,
    sigmaSpeed: 0,
    warnings: [
      "No measured calibration. Physics values are editable starting estimates.",
    ],
    diagnostics: [],
    aeroFitted: false,
  },
  comparison: [],
});
export const rad = (d: number) => (d * Math.PI) / 180;
export const deg = (r: number) => (r * 180) / Math.PI;
export const finite = (...x: number[]) => x.every(Number.isFinite);
