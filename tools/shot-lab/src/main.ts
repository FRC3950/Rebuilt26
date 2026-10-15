import "./style.css";
import {
  defaults,
  rad,
  deg,
  type Project,
  type Scenario,
  type Shot,
  type Trial,
  type Physics,
} from "./types";
import { load, save, parseProject } from "./storage";
import {
  calibrate,
  fitAerodynamics,
  mapValue,
  conditionalMap,
} from "./calibration";
import { simulate, turretState } from "./physics";
import {
  solveShot,
  trace,
  evaluateModel,
  exportModel,
  shotPhysics,
  spinAt,
  releaseRps,
  type Candidate,
} from "./solver";
import { readLog, snapshot, csvRows, csvEncode, type Log } from "./logs";
let project: Project;
let storageIssue = "";
try {
  project = load(localStorage);
} catch (e) {
  project = defaults();
  storageIssue = (e as Error).message;
}
let tab = "explore",
  worker: Worker | null = null,
  log: Log | null = null,
  shot: Shot | null = null,
  candidates: Candidate[] = [],
  progress = 0,
  jobMessage = "Ready for measurements";
let scenario: Scenario = { distance: 3.5, radial: 0, tangential: 0 };
let angle = 65,
  speed = 9,
  spin = 15,
  yaw = 0,
  frame = 1;
let recoveryPending = !!storageIssue;
const $ = <T extends HTMLElement = HTMLElement>(s: string) =>
  document.querySelector<T>(s)!;
const escape = (x: unknown) =>
  String(x ?? "").replace(
    /[&<>"']/g,
    (c) =>
      ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[
        c
      ]!,
  );
function persist(explicit = true) {
  try {
    if (recoveryPending) {
      if (!explicit) return;
      const previous = localStorage.getItem("chuck-shot-lab-v1");
      if (previous)
        localStorage.setItem("chuck-shot-lab-recovery-" + Date.now(), previous);
      recoveryPending = false;
    }
    save(localStorage, project);
    $("#save-state").textContent = "Saved on this laptop";
  } catch (e) {
    toast("Local save failed. Export the project to keep your work.");
  }
}
function invalidate() {
  if (worker) {
    worker.terminate();
    worker = null;
    jobMessage = "Cancelled because inputs changed.";
  }
  project.model = undefined;
  shot = null;
  persist();
}
function toast(s: string) {
  const e = $("#toast");
  e.textContent = s;
  e.hidden = false;
  setTimeout(() => (e.hidden = true), 8000);
}
function download(name: string, data: unknown, type = "application/json") {
  const blob = new Blob(
    [typeof data === "string" ? data : JSON.stringify(data, null, 2)],
    { type },
  );
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}
function shell() {
  $("#app").innerHTML =
    `<aside><a class="brand" href="#"><span class="brand-icon">C<span>↗</span></span><span>CHUCK NORRIS<small>SHOT LAB / 2026</small></span></a><div class="nav-label">WORKSPACE</div><nav>${[
      ["explore", "◎", "Trajectory studio"],
      ["measure", "⊞", "Measurements"],
      ["physics", "⌁", "Robot & physics"],
      ["fit", "◈", "Fit & coverage"],
      ["logs", "≋", "Robot logs"],
    ]
      .map(
        ([id, icon, label]) =>
          `<button data-tab="${id}" class="${tab === id ? "active" : ""}"><span>${icon}</span>${label}</button>`,
      )
      .join(
        "",
      )}</nav><div class="sidebar-bottom"><span class="status-dot"></span> OFFLINE WORKSPACE<p>Team 3950 · Rebuilt26<br>Two turrets. One measured model.</p><button id="save-project" class="secondary">↓ Export project</button><label class="file secondary">↑ Open project<input id="open-project" type="file" accept=".json"></label></div></aside><main><header><div><span class="eyebrow">CHUCK NORRIS / ALLIANCE ZONE</span><h1>${({ explore: "Make every shot count.", measure: "Measure. Then model.", physics: "Built around your robot.", fit: "From physics to polynomials.", logs: "Put the shot in context." } as Record<string, string>)[tab]}</h1></div><div class="save"><span id="save-state">Saved on this laptop</span><span class="pill ${project.calibration.measured ? "green" : ""}">${project.calibration.measured ? "Measured transfer" : "Awaiting calibration"}</span></div></header><div class="portable-tools"><button class="secondary" id="save-project-small">↓ Export project</button><label class="file secondary">↑ Open project<input id="open-project-small" type="file" accept=".json"></label></div><p class="notice">Advanced prototype reference. The current robot uses the version-2 radial quadratic map. Older version-1 exports from this page are incompatible. <a href="map.html">Open the robot map generator</a>. <a href="/">Open the simple tuning guide</a>.</p><section id="content"></section><footer>CHUCK SHOT LAB <span>SI units · Local files · No cloud services</span></footer></main><div id="toast" role="status" hidden></div>`;
  document.querySelectorAll<HTMLButtonElement>("[data-tab]").forEach(
    (b) =>
      (b.onclick = () => {
        tab = b.dataset.tab!;
        render();
      }),
  );
  $("#save-project").onclick = () =>
    download("chuck-shot-lab.project.json", project);
  $("#save-project-small").onclick = $("#save-project").onclick;
  $("#open-project").onchange = async (e) => {
    try {
      const file = (e.target as HTMLInputElement).files?.[0];
      if (file) {
        const next = parseProject(await file.text());
        worker?.terminate();
        worker = null;
        jobMessage = "Ready for measurements";
        project = next;
        persist();
        render();
        toast("Project opened.");
      }
    } catch (e) {
      toast((e as Error).message);
    }
  };
  $("#open-project-small").onchange = $("#open-project").onchange;
}
const number = (id: string, label: string, value: number, step = 0.1) =>
  `<label class="control">${label}<input id="${id}" type="number" step="${step}" value="${value}"></label>`;
const title = (kicker: string, name: string, extra = "") =>
  `<div class="panel-title"><div><span class="eyebrow">${kicker}</span><h2>${name}</h2></div>${extra}</div>`;
function render() {
  shell();
  (({ explore, measure, physics, fit, logs })[tab] ?? explore)();
  if (storageIssue) {
    toast(
      `Saved project could not be loaded: ${storageIssue}. Original storage remains until you save.`,
    );
    storageIssue = "";
  }
}
function explore() {
  $("#content").innerHTML =
    `<div class="stats"><div><span>ALLIANCE ZONE</span><strong>5.18 <small>× 8.08 m</small></strong></div><div><span>SHOOTING MOTION</span><strong>${project.geometry.maxSpeed.toFixed(1)} <small>m/s + rotation</small></strong></div><div><span>MEASURED TRIALS</span><strong>${project.trials.length.toString().padStart(2, "0")} <small>recorded</small></strong></div><div><span>MODEL STATUS</span><strong class="small">${project.model?.validation.passed ? "Sim validated" : project.model ? "Coverage gaps" : "Not fitted"}</strong></div></div><div class="studio"><div class="panel trajectory">${title("01 / FLIGHT ENGINE", "Trajectory studio", '<span class="pill">Exploration · uncalibrated inputs</span>')}<div class="canvas-wrap"><canvas id="trajectory" width="1100" height="460" aria-label="Side view of the ball trajectory and hub"></canvas><div class="plot-key"><i></i> FLIGHT PATH <i class="hub-key"></i> HUB OPENING <span id="flight-result"></span></div></div><div class="playback"><span>RELEASE</span><input id="frame" type="range" min="0" max="1" step="0.01" value="${frame}" aria-label="Trajectory playback"><span>FLIGHT END</span></div><div class="panel-sub"><span id="flight-detail"></span><button class="secondary" id="optimal">Find robust shot</button></div></div><div class="panel flight-controls">${title("SCENARIO", "Set up a shot")}<p class="muted">Physical release inputs. These estimates are never exported as measured calibration.</p><div class="inputs">${number("distance", "Distance to hub · m", scenario.distance)}${number("radial", "Velocity toward hub · m/s", scenario.radial)}${number("tangential", "Velocity across hub · m/s", scenario.tangential)}${number("angle", "Launch elevation · °", angle)}${number("speed", "Ball exit speed · m/s", speed)}${number("spin", "Backspin · rev/s", spin)}${number("yaw", "Aim offset · °", deg(yaw))}${number("azimuth", "Hub bearing · field °", deg(scenario.azimuth ?? 0))}</div><button id="simulate">Simulate trajectory ↗</button></div></div><div class="two-up"><div class="panel">${title("02 / FIELD VIEW", "Choose your position", '<span class="muted">Click to set distance</span>')}<canvas id="field" width="680" height="400" aria-label="Interactive alliance zone and coverage map"></canvas><p class="muted">Blue-origin coordinates. Hub structure excluded from driving positions. Both turret offsets are included in model validation.</p></div><div class="panel">${title("03 / SHOT WINDOW", "Feasible release region")}<canvas id="feasible" width="680" height="350" aria-label="Feasible launch elevation and speed"></canvas><p class="muted">${project.calibration.measured ? "Run “Find robust shot” to compare scoring candidates and asymmetric margins." : "Collect repeated release-angle, exit-speed and backspin measurements to unlock the feasible-region search."}</p><div id="margins"></div></div></div>`;
  const simulateNow = () => {
    try {
      scenario = {
        distance: read("distance"),
        radial: read("radial"),
        tangential: read("tangential"),
        azimuth: rad(read("azimuth")),
      };
      angle = read("angle");
      speed = read("speed");
      spin = read("spin");
      yaw = rad(read("yaw"));
      drawFlight();
      drawField();
    } catch (e) {
      toast((e as Error).message);
    }
  };
  $("#simulate").onclick = simulateNow;
  $("#frame").oninput = () => {
    frame = read("frame");
    drawFlight();
  };
  $("#optimal").onclick = () => {
    if (!project.calibration.measured) {
      toast("First fit measured transfer data in Measurements.");
      return;
    }
    startWorker("solve");
  };
  $("#field").onclick = (e) => {
    const c = $<HTMLCanvasElement>("#field"),
      r = c.getBoundingClientRect(),
      x = ((e.clientX - r.left) / r.width) * 6,
      y = (1 - (e.clientY - r.top) / r.height) * 8.1;
    scenario.azimuth = Math.atan2(
      project.geometry.hub[1] - y,
      project.geometry.hub[0] - x,
    );
    $<HTMLInputElement>("#azimuth").value = deg(scenario.azimuth).toFixed(2);
    scenario.distance = Math.hypot(
      x - project.geometry.hub[0],
      y - project.geometry.hub[1],
    );
    $<HTMLInputElement>("#distance").value = scenario.distance.toFixed(2);
    simulateNow();
  };
  drawFlight();
  drawField();
  drawFeasible();
}
function read(id: string) {
  const n = Number($<HTMLInputElement>("#" + id).value);
  if (!Number.isFinite(n)) throw Error(`Invalid ${id}`);
  return n;
}
function context(id: string) {
  const c = $<HTMLCanvasElement>("#" + id),
    ctx = c.getContext("2d")!;
  ctx.clearRect(0, 0, c.width, c.height);
  ctx.font = "13px system-ui";
  return { c, ctx };
}
function drawFlight() {
  const f = simulate(
      shotPhysics(project, scenario.azimuth ?? 0),
      scenario,
      angle,
      speed,
      spin,
      yaw,
    ),
    { c, ctx } = context("trajectory");
  const xmax = Math.max(6, scenario.distance + 1, ...f.path.map((p) => p[0])),
    zmax = Math.max(3, ...f.path.map((p) => p[2])) + 0.3;
  const px = (x: number) => 55 + (x / xmax) * (c.width - 90),
    py = (z: number) => c.height - 45 - (z / zmax) * (c.height - 75);
  ctx.strokeStyle = "#263e43";
  ctx.fillStyle = "#829b9d";
  for (let i = 0; i <= 5; i++) {
    const z = (zmax * i) / 5;
    ctx.beginPath();
    ctx.moveTo(55, py(z));
    ctx.lineTo(c.width - 30, py(z));
    ctx.stroke();
    ctx.fillText(z.toFixed(1) + " m", 8, py(z) + 4);
  }
  for (let i = 0; i <= 6; i++)
    ctx.fillText(
      ((xmax * i) / 6).toFixed(1),
      px((xmax * i) / 6) - 8,
      c.height - 18,
    );
  const p = project.physics;
  ctx.fillStyle = "#2a4549";
  ctx.fillRect(
    px(scenario.distance - p.hubRadius),
    py(p.hubHeight),
    px(2 * p.hubRadius) - px(0),
    py(0) - py(p.hubHeight),
  );
  ctx.strokeStyle = "#83a7aa";
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(px(scenario.distance - p.hubRadius), py(p.hubHeight));
  ctx.lineTo(px(scenario.distance + p.hubRadius), py(p.hubHeight));
  ctx.stroke();
  ctx.fillStyle = "#9bb9bb";
  ctx.fillText("HUB", px(scenario.distance) - 13, py(p.hubHeight) - 18);
  ctx.strokeStyle = "#c4f369";
  ctx.lineWidth = 3;
  ctx.beginPath();
  const end = Math.floor((f.path.length - 1) * frame);
  f.path
    .slice(0, end + 1)
    .forEach((v, i) =>
      i ? ctx.lineTo(px(v[0]), py(v[2])) : ctx.moveTo(px(v[0]), py(v[2])),
    );
  ctx.stroke();
  const ball = f.path[end];
  ctx.fillStyle = "#d4ff80";
  ctx.shadowColor = "#bbf66a";
  ctx.shadowBlur = 16;
  ctx.beginPath();
  ctx.arc(px(ball[0]), py(ball[2]), 7, 0, Math.PI * 2);
  ctx.fill();
  ctx.shadowBlur = 0;
  $("#flight-result").textContent = f.scored
    ? "CLEAN ENTRY"
    : f.collision
      ? "RIM CONTACT"
      : "MISS";
  $("#flight-result").className = f.scored ? "good" : "muted";
  $("#flight-detail").textContent =
    `${f.tof.toFixed(3)} s to hub plane · apex ${Math.max(...f.path.map((p) => p[2])).toFixed(2)} m · cross-track ${f.crossing?.[1].toFixed(3) ?? "—"} m`;
}
function drawField() {
  const { c, ctx } = context("field"),
    px = (x: number) => (x / 6) * c.width,
    py = (y: number) => (1 - y / 8.1) * c.height;
  ctx.fillStyle = "#152a30";
  ctx.fillRect(0, 0, c.width, c.height);
  ctx.strokeStyle = "#30464a";
  for (let i = 0; i < 9; i++) {
    ctx.beginPath();
    ctx.moveTo(0, py(i));
    ctx.lineTo(c.width, py(i));
    ctx.stroke();
  }
  for (let i = 0; i < 7; i++) {
    ctx.beginPath();
    ctx.moveTo(px(i), 0);
    ctx.lineTo(px(i), c.height);
    ctx.stroke();
  }
  for (const v of project.model?.validation.coverage ?? []) {
    ctx.fillStyle = v.passed ? "#bfee6c80" : "#f5826880";
    ctx.beginPath();
    ctx.arc(px(v.x), py(v.y), 4, 0, Math.PI * 2);
    ctx.fill();
  }
  const [hx, hy] = project.geometry.hub;
  ctx.fillStyle = "#b8d5d4";
  ctx.fillRect(px(hx - 0.56), py(hy + 0.56), px(1.12), c.height - py(1.12));
  ctx.fillStyle = "#10262b";
  ctx.fillText("HUB", px(hx) - 15, py(hy) + 4);
  ctx.setLineDash([5, 6]);
  ctx.strokeStyle = "#c4f36966";
  ctx.beginPath();
  ctx.ellipse(
    px(hx),
    py(hy),
    px(scenario.distance),
    (scenario.distance / 8.1) * c.height,
    0,
    0,
    Math.PI * 2,
  );
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = "#c4f369";
  ctx.beginPath();
  ctx.arc(
    px(hx - scenario.distance * Math.cos(scenario.azimuth ?? 0)),
    py(hy - scenario.distance * Math.sin(scenario.azimuth ?? 0)),
    7,
    0,
    Math.PI * 2,
  );
  ctx.fill();
}
function drawFeasible() {
  const { c, ctx } = context("feasible");
  const lo = 30,
    hi = 85,
    vlo = 0,
    vhi = 20;
  ctx.strokeStyle = "#30464a";
  ctx.fillStyle = "#829b9d";
  for (let i = 0; i < 5; i++) {
    const y = 30 + i * 65;
    ctx.beginPath();
    ctx.moveTo(40, y);
    ctx.lineTo(c.width - 20, y);
    ctx.stroke();
    ctx.fillText(20 - i * 5 + " m/s", 3, y - 5);
  }
  ctx.fillText("Launch elevation →", c.width / 2 - 50, c.height - 5);
  for (const p of candidates) {
    ctx.fillStyle = p.scored ? "#c4f369" : "#f58268";
    ctx.beginPath();
    ctx.arc(
      40 + ((p.angle - lo) / (hi - lo)) * (c.width - 60),
      290 - ((p.speed - vlo) / (vhi - vlo)) * 260,
      3,
      0,
      Math.PI * 2,
    );
    ctx.fill();
    if (p.minimumSpeed !== undefined && p.maximumSpeed !== undefined) {
      ctx.strokeStyle = "#c4f36988";
      ctx.lineWidth = 2;
      ctx.beginPath();
      const x = 40 + ((p.angle - lo) / (hi - lo)) * (c.width - 60);
      ctx.moveTo(x, 290 - (p.minimumSpeed / vhi) * 260);
      ctx.lineTo(x, 290 - (p.maximumSpeed / vhi) * 260);
      ctx.stroke();
    }
  }
  if (!candidates.length) {
    ctx.fillStyle = "#829b9d";
    ctx.fillText("MEASUREMENTS → FEASIBLE SHOTS → ROBUST FIT", 70, 170);
  }
  if (shot)
    $("#margins").innerHTML =
      `<div class="margin-grid">${shot.margins.map((m, i) => `<div><span>${["Angle −", "Angle +", "Speed −", "Speed +"][i]}</span><strong>${m.toFixed(3)} ${i < 2 ? "°" : "m/s"}</strong></div>`).join("")}</div>`;
}
const trialKeys = [
  "id",
  "turret",
  "timestamp",
  "hood",
  "rps",
  "commandRps",
  "elevation",
  "speed",
  "spin",
  "tof",
  "distance",
  "radial",
  "tangential",
  "yaw",
  "azimuth",
  "landingX",
  "landingY",
  "outcome",
  "notes",
];
function measure() {
  $("#content").innerHTML =
    `<div class="notice"><span>01 → 02 → 03</span><div><strong>Calibrate the release before fitting the shot.</strong><p>Measure actual ball elevation, exit speed and backspin at repeated hood/RPS settings. Then record varied flight times and crossing positions to identify drag and lift. Hood feedback is a command estimate, not an angle sensor.</p></div></div><div class="two-up"><div class="panel">${title("01 / NEW TRIAL", "Record measured evidence")}<form id="trial-form"><div class="inputs three">${[
      ["hood", "Hood command · °"],
      ["rps", "Measured wheel · RPS"],
      ["commandRps", "Commanded wheel · RPS"],
      ["elevation", "Measured elevation · °"],
      ["speed", "Measured ball speed · m/s"],
      ["spin", "Measured backspin · rev/s"],
      ["tof", "Measured flight time · s"],
      ["distance", "Target distance · m"],
      ["radial", "Radial motion · m/s"],
      ["tangential", "Lateral motion · m/s"],
      ["yaw", "Release yaw offset · rad"],
      ["azimuth", "Hub bearing · field rad"],
      ["landingX", "Crossing / landing X · m"],
      ["landingY", "Crossing / landing Y · m"],
    ]
      .map(
        ([id, label]) =>
          `<label class="control">${label}<input name="${id}" type="number" step="any" ${["hood", "rps"].includes(id) ? "required" : ""}></label>`,
      )
      .join(
        "",
      )}<label class="control">Turret<select name="turret"><option>Left</option><option>Right</option></select></label><label class="control">Outcome<select name="outcome"><option value="">Not observed</option><option>score</option><option>miss</option><option>rim</option><option>ground</option></select></label><label class="control">Notes<input name="notes"></label></div><button>Add measured trial +</button></form><p class="muted">X/Y are measured from the release point in hub-aligned coordinates. Use the hub-height crossing time/position, or choose “ground” for ground-height observations. Leave unmeasured fields blank.</p></div><div class="panel">${title("02 / CALIBRATION", "Transfer & aerodynamic fit")}<div class="stack"><button id="fit-transfer">Fit measured release transfer</button><button id="fit-aero" class="secondary">Fit drag, Magnus & spin decay</button><label class="file secondary">Import measurement CSV<input id="csv-import" type="file" accept=".csv"></label><button class="secondary" id="trial-template">Download guided trial worksheet</button><button class="secondary" id="trials-export">Export measured trials</button></div><div class="diagnostics">${project.calibration.warnings.map((w) => `<p class="warning">${escape(w)}</p>`).join("")}${project.calibration.diagnostics.map((d) => `<div><span>${escape(d.parameter)}</span><strong>${d.value.toPrecision(4)} ${d.uncertainty === null ? "· not identifiable" : `± ${d.uncertainty.toPrecision(2)}`}</strong><small>${escape(d.source)}</small></div>`).join("")}</div><p class="muted">Compression and slip are captured by measured RPS → speed/spin transfer. Hood-dependent speed/spin maps capture changing contact and compression. Wheel repeats are grouped by commanded RPS, or by 1 RPS bins when commands are absent. Separating compression from slip requires additional contact measurements.</p></div></div><div class="panel">${title("03 / EVIDENCE", "Trial ledger", `<span class="pill">${project.trials.length} trials</span>`)}<div class="table-scroll"><table><thead><tr>${["ID / source", "Turret", "Hood °", "Wheel RPS", "Release °", "Exit m/s", "Spin rev/s", "TOF s", "Outcome", ""].map((x) => `<th>${x}</th>`).join("")}</tr></thead><tbody>${project.trials.map((r, i) => `<tr><td>${escape(r.id)}<small>${escape(r.source ?? "Manual measurement")}</small></td>${[r.turret, r.hood, r.rps, r.elevation, r.speed, r.spin, r.tof, r.outcome].map((x) => `<td>${escape(x ?? "—")}</td>`).join("")}<td><button class="text" data-edit="${i}">Edit</button><button class="text" data-delete="${i}">Remove</button></td></tr>`).join("") || '<tr><td colspan="10" class="empty">No measurements yet. Add a trial or import your worksheet.</td></tr>'}</tbody></table></div></div>`;
  const reference = document.createElement("details");
  reference.className = "panel";
  reference.innerHTML = `<summary>Existing Rebuilt26 shot table · comparison only</summary><p class="muted">These rows are never used to fit measured release calibration.</p><div class="table-scroll"><table><thead><tr><th>Distance · m</th><th>Hood command · °</th><th>Flywheel · RPS</th><th>Flight time · s</th></tr></thead><tbody>${project.comparison.map((r) => `<tr><td>${r.d}</td><td>${r.hoodDeg}</td><td>${r.rps}</td><td>${r.tof}</td></tr>`).join("")}</tbody></table></div>`;
  $("#content").append(reference);
  $("#trial-form").onsubmit = (e) => {
    e.preventDefault();
    try {
      const data = new FormData(e.target as HTMLFormElement),
        r: Record<string, unknown> = {
          id:
            (e.target as HTMLFormElement).dataset.trialId ??
            crypto.randomUUID().slice(0, 8),
          source:
            (e.target as HTMLFormElement).dataset.source ??
            "Manual measurement",
        };
      for (const [k, v] of data)
        if (v !== "")
          r[k] = ["turret", "outcome", "notes"].includes(k) ? v : Number(v);
      addTrials([r as unknown as Trial]);
      render();
      toast("Measured trial recorded.");
    } catch (e) {
      toast((e as Error).message);
    }
  };
  $("#fit-transfer").onclick = () => {
    project.calibration = calibrate(project.trials);
    invalidate();
    render();
    toast(
      project.calibration.measured
        ? "Measured release transfer fitted."
        : "More measurements needed; see calibration diagnostics.",
    );
  };
  $("#fit-aero").onclick = () => {
    try {
      if (!project.calibration.measured)
        throw Error("Fit measured release transfer first.");
      const r = fitAerodynamics(project.trials, project.physics);
      project.physics = r.physics;
      project.calibration.diagnostics = project.calibration.diagnostics.filter(
        (d) => !["drag", "magnus", "spinDecay"].includes(d.parameter),
      );
      project.calibration.diagnostics.push(...r.diagnostics);
      project.calibration.aeroFitted = true;
      invalidate();
      render();
      toast(
        `Flight fit RMSE ${r.rmse.toFixed(4)} m. Inspect uncertainties before proceeding.`,
      );
    } catch (e) {
      toast((e as Error).message);
    }
  };
  $("#csv-import").onchange = async (e) => {
    try {
      const f = (e.target as HTMLInputElement).files?.[0];
      if (!f) return;
      const rows = csvRows(await f.text())
        .filter((r) => r.hood !== "" && r.rps !== "")
        .map((row, i) => {
          const r: Record<string, unknown> = {
            id: row.id || `${f.name}-${i + 1}`,
            source: f.name,
          };
          for (const k of trialKeys) {
            if (row[k] !== undefined && row[k] !== "")
              r[k] = ["id", "turret", "outcome", "notes"].includes(k)
                ? row[k]
                : Number(row[k]);
          }
          return r as unknown as Trial;
        });
      addTrials(rows);
      render();
      toast(`${rows.length} trials imported.`);
    } catch (e) {
      toast((e as Error).message);
    }
  };
  $("#trial-template").onclick = () => {
    const rows: Record<string, unknown>[] = [];
    for (const turret of ["Left", "Right"])
      for (const hood of project.geometry.hood.concat([
        (project.geometry.hood[0] + project.geometry.hood[1]) / 2,
      ]))
        for (const rps of [35, 50, 65, 80, 95])
          for (let i = 0; i < 3; i++)
            rows.push({
              id: `${turret}-${hood}-${rps}-${i + 1}`,
              turret,
              hood,
              rps,
              notes:
                "Replace RPS with measured steady wheel speed; measure release elevation/speed/spin. Add flight observations at varied distances.",
            });
    download(
      "chuck-measurement-worksheet.csv",
      csvEncode(rows, trialKeys),
      "text/csv",
    );
  };
  $("#trials-export").onclick = () =>
    download(
      "chuck-measurements.csv",
      csvEncode(
        project.trials as unknown as Record<string, unknown>[],
        trialKeys,
      ),
      "text/csv",
    );
  document.querySelectorAll<HTMLButtonElement>("[data-edit]").forEach(
    (b) =>
      (b.onclick = () => {
        const trial = project.trials[Number(b.dataset.edit)],
          form = $<HTMLFormElement>("#trial-form");
        for (const key of trialKeys) {
          const input = form.elements.namedItem(key) as HTMLInputElement | null;
          if (input)
            input.value = String(
              (trial as unknown as Record<string, unknown>)[key] ?? "",
            );
        }
        form.dataset.trialId = trial.id;
        form.dataset.source = trial.source ?? "Manual measurement";
        form.querySelector("button")!.textContent = "Save edited trial";
        form.scrollIntoView({ behavior: "smooth" });
        toast("Edit the measured fields, then save the trial.");
      }),
  );
  document.querySelectorAll<HTMLButtonElement>("[data-delete]").forEach(
    (b) =>
      (b.onclick = () => {
        project.trials.splice(Number(b.dataset.delete), 1);
        project.calibration = calibrate(project.trials);
        invalidate();
        render();
      }),
  );
}
function addTrials(rows: Trial[]) {
  for (const r of rows) {
    if (
      !Number.isFinite(r.hood) ||
      !Number.isFinite(r.rps) ||
      Object.values(r).some((x) => typeof x === "number" && !Number.isFinite(x))
    )
      throw Error("CSV/trial contains invalid numbers.");
  }
  for (const r of rows) {
    const i = project.trials.findIndex((x) => x.id === r.id);
    if (i < 0) project.trials.push(r);
    else project.trials[i] = { ...project.trials[i], ...r };
  }
  project.calibration = calibrate(project.trials);
  invalidate();
}
const physicsLabels: Record<string, string> = {
  gravity: "Gravity · m/s²",
  density: "Air density · kg/m³",
  drag: "Drag coefficient",
  magnus: "Magnus coefficient",
  spinDecay: "Spin decay · 1/s",
  mass: "Ball mass · kg",
  radius: "Ball radius · m",
  launchHeight: "Release height · m",
  hubHeight: "Hub opening height · m",
  hubRadius: "Rim center radius · m",
  rimRadius: "Rim tube radius · m",
  restitution: "Rim restitution",
  dt: "Integration step · s",
  latency: "Release latency · s",
  speedScale: "Speed transfer multiplier",
  spinScale: "Spin transfer multiplier",
  angleOffset: "Release elevation offset · °",
};
function physics() {
  $("#content").innerHTML =
    `<div class="notice"><span>PARAMETER PROVENANCE</span><div><strong>Starting geometry comes from Rebuilt26. Flight parameters need measurement.</strong><p>Release height and hub dimensions are simulator estimates. Angle/speed transfer is measured independently. Editing parameters invalidates the exported fit.</p></div></div><form id="physics-form"><div class="panel">${title("01 / BALL FLIGHT", "Physics & release")}<div class="inputs four">${Object.entries(
      physicsLabels,
    )
      .map(([k, l]) =>
        number(
          "p-" + k,
          l,
          project.physics[k as keyof Physics] as number,
          0.001,
        ),
      )
      .join(
        "",
      )}${project.physics.wind.map((v, i) => number("wind-" + i, `Wind ${["X", "Y", "Z"][i]} · m/s`, v)).join("")}</div></div><div class="panel">${title("02 / CHUCK NORRIS", "Geometry & mechanism limits")}<div class="inputs four">${project.geometry.hood.map((v, i) => number("hood-" + i, `${i ? "Maximum" : "Minimum"} hood command · °`, v)).join("")}${project.geometry.rps.map((v, i) => number("rps-" + i, `${i ? "Maximum" : "Minimum"} flywheel · RPS`, v)).join("")}${number("maxSpeed", "Shooting speed limit · m/s", project.geometry.maxSpeed)}${number("maxOmega", "Angular speed limit · rad/s", project.geometry.maxOmega)}${number("maxTilt", "Tilt validation bound · °", project.geometry.maxTilt)}${project.geometry.hub.map((v, i) => number("hub-" + i, `Blue hub ${["X", "Y"][i]} · m`, v)).join("")}${project.geometry.turrets.flatMap((t, i) => t.map((v, j) => number(`turret-${i}-${j}`, `${i ? "Right" : "Left"} turret ${["X", "Y"][j]} · m`, v, 0.0001))).join("")}${project.geometry.zone.map((v, i) => number("zone-" + i, `Alliance zone ${["min X", "max X", "min Y", "max Y"][i]} · m`, v)).join("")}</div><button>Save parameters & invalidate fit</button></div></form>`;
  $("#physics-form").onsubmit = (e) => {
    e.preventDefault();
    try {
      const next = structuredClone(project);
      for (const k of Object.keys(physicsLabels))
        (next.physics as unknown as Record<string, unknown>)[k] = read(
          "p-" + k,
        );
      next.physics.wind = next.physics.wind.map((_, i) =>
        read("wind-" + i),
      ) as [number, number, number];
      for (const k of ["hood", "rps", "hub", "zone"] as const)
        next.geometry[k] = next.geometry[k].map((_, i) =>
          read(`${k}-${i}`),
        ) as never;
      for (const k of ["maxSpeed", "maxOmega", "maxTilt"] as const)
        next.geometry[k] = read(k);
      next.geometry.turrets = next.geometry.turrets.map(
        (t, i) => t.map((_, j) => read(`turret-${i}-${j}`)) as [number, number],
      );
      parseProject(JSON.stringify(next));
      if (
        next.physics.drag !== project.physics.drag ||
        next.physics.magnus !== project.physics.magnus ||
        next.physics.spinDecay !== project.physics.spinDecay
      )
        next.calibration.aeroFitted = false;
      project = next;
      invalidate();
      render();
      toast("Parameters saved. Generate and validate a new fit.");
    } catch (e) {
      toast((e as Error).message);
    }
  };
}
function fit() {
  const v = project.model?.validation;
  $("#content").innerHTML =
    `<div class="notice"><span>MEASURE → SIMULATE → FIT → VALIDATE</span><div><strong>Full alliance-zone coverage is the goal.</strong><p>The generator finds robust scoring regions, fits seven outputs, then subdivides quadratic patches where the fit exceeds 10% of the measured scoring margin. Every gap remains visible.</p></div></div><div class="two-up"><div class="panel">${title("01 / OFFLINE GENERATOR", "Build the compact model")}<div class="checklist"><p>${project.calibration.measured ? "●" : "○"} Measured release mapping and repeatability</p><p>${project.calibration.aeroFitted ? "●" : "○"} Identifiable aerodynamic flight fit</p><p>${v?.passed ? "●" : "○"} Fit error and trajectory validation</p><p>○ Physical validation across the alliance zone</p></div><div class="button-row"><button id="generate" ${worker ? "disabled" : ""}>Generate & validate model ↗</button><button id="cancel" class="secondary" ${worker ? "" : "disabled"}>Cancel</button></div><progress id="progress" value="${progress}" max="1"></progress><p id="job-message" class="muted">${escape(jobMessage)}</p><p class="muted">Runs in a dedicated worker. You can use other tabs while the sweep runs. Cancel terminates it without replacing your saved model.</p></div><div class="panel">${title("02 / EXPORT", "Robot model & evidence")}<div class="stack"><button id="export-model" ${v?.passed ? "" : "disabled"}>↓ Export chuck_shot_model.json</button><button id="export-table" class="secondary" ${v?.passed ? "" : "disabled"}>↓ Export stationary shot_table.json</button><button id="export-report" class="secondary" ${v ? "" : "disabled"}>↓ Export validation report</button></div><p class="muted">After calibration and validation, copy the model to Rebuilt26/src/main/deploy/chuck_shot_model.json and restart the robot program after deployment. This app never deploys it.</p><div class="stats compact"><div><span>QUADRATIC PATCHES</span><strong>${project.model?.patches.length ?? "—"}</strong></div><div><span>CHECKED SCENARIOS</span><strong>${v?.samples.toLocaleString() ?? "—"}</strong></div><div><span>MAX FIT / MARGIN</span><strong>${v ? (v.maxMarginFraction * 100).toFixed(2) + "%" : "—"}</strong></div><div><span>COVERAGE GAPS</span><strong>${v?.failures ?? "—"}</strong></div></div></div></div><div class="two-up"><div class="panel">${title("03 / ALLIANCE ZONE", "Coverage map")}<canvas id="field" width="680" height="400"></canvas><p class="muted">Green: all tested motions pass. Coral: unresolved coverage. Both turrets, four headings, eight translation directions and ± maximum rotation. No map is shown until validation runs.</p></div><div class="panel">${title("04 / DIAGNOSTICS", "Validation findings")}<canvas id="fit-errors" width="680" height="260" aria-label="Polynomial errors as fractions of scoring margins"></canvas><div class="issues">${v?.issues.length ? v.issues.map((x) => `<p class="warning">${escape(x)}</p>`).join("") : '<p class="muted">No validation issues reported. A passing simulated model remains distinct from physical shot validation.</p>'}</div><p class="muted">Fit basis: 1, distance, radial speed, distance², distance × radial, radial². Signed tangential layers carry fitted aim corrections. Wind adds field-bearing layers. Interpolated trajectories are re-simulated during coverage validation.</p></div></div>`;
  $("#generate").onclick = () => startWorker("generate");
  $("#cancel").onclick = () => {
    worker?.terminate();
    worker = null;
    jobMessage = "Cancelled. Saved model unchanged.";
    progress = 0;
    render();
  };
  $("#export-model").onclick = () => {
    try {
      download("chuck_shot_model.json", exportModel(project));
    } catch (e) {
      toast((e as Error).message);
    }
  };
  $("#export-table").onclick = () => {
    try {
      const m = exportModel(project),
        points = [];
      const end = Math.max(...m.patches.map((p) => p.bounds[1]));
      for (let d = 0.65; d <= end; d += 0.1) {
        const s = evaluateModel(m, { distance: d, radial: 0, tangential: 0 });
        points.push({
          d: +d.toFixed(2),
          hoodDeg: mapValue(
            m.calibration.angle,
            s.angle - m.physics.angleOffset,
            true,
          ),
          rps: releaseRps(project, s.speed, s.angle),
          tof: s.tof,
        });
      }
      download("shot_table.json", {
        units: { distance: "m", hood: "deg", flywheel: "rps", tof: "s" },
        points,
      });
    } catch (e) {
      toast((e as Error).message);
    }
  };
  $("#export-report").onclick = () =>
    download("chuck-validation-report.json", {
      name: project.name,
      createdAt: project.model?.createdAt,
      calibration: project.calibration,
      validation: v,
    });
  drawField();
  drawFitErrors();
}
function drawFitErrors() {
  const { c, ctx } = context("fit-errors"),
    points = project.model?.validation.residuals ?? [];
  ctx.fillStyle = "#8fa8aa";
  ctx.fillText("Error / scoring margin · threshold 10%", 20, 20);
  const max = Math.max(
    0.2,
    ...points.map((p) =>
      Math.max(p.angleFraction, p.speedFraction, p.marginFraction),
    ),
  );
  const y = (v: number) => c.height - 30 - (v / max) * (c.height - 65);
  ctx.strokeStyle = "#f0ba86";
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(25, y(0.1));
  ctx.lineTo(c.width - 20, y(0.1));
  ctx.stroke();
  ctx.setLineDash([]);
  points.forEach((p, i) => {
    const x = 30 + (i / Math.max(1, points.length - 1)) * (c.width - 60);
    for (const [value, color] of [
      [p.angleFraction, "#c4f369"],
      [p.speedFraction, "#8bc8d0"],
      [p.marginFraction, "#f0ba86"],
    ] as [number, string][]) {
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(x, y(value), 2, 0, Math.PI * 2);
      ctx.fill();
    }
  });
  if (!points.length)
    ctx.fillText("Generate a measured model to inspect residuals.", 55, 140);
}
function startWorker(kind: string) {
  if (worker) {
    toast("A sweep is already running.");
    return;
  }
  if (!project.calibration.measured) {
    toast("Fit measured release transfer first.");
    return;
  }
  if (kind === "generate" && !project.calibration.aeroFitted) {
    toast("Fit measured aerodynamic flight data first.");
    return;
  }
  const requestedScenario = structuredClone(scenario);
  worker = new Worker(new URL("./worker.ts", import.meta.url), {
    type: "module",
  });
  jobMessage = "Starting simulation…";
  progress = 0;
  worker.onmessage = ({ data }) => {
    if (data.kind === "progress") {
      progress = data.fraction;
      jobMessage = data.message;
      if (tab === "fit") {
        $<HTMLProgressElement>("#progress").value = progress;
        $("#job-message").textContent = jobMessage;
      }
      return;
    }
    worker?.terminate();
    worker = null;
    if (data.kind === "error") {
      jobMessage = data.message;
      toast(data.message);
      if (tab === "fit") render();
      return;
    }
    if (kind === "generate") {
      project.model = data.result;
      persist();
      progress = 1;
      jobMessage = project.model?.validation.passed
        ? "Simulation and coverage checks passed. Ready for robot export."
        : "Coverage gaps found. Review diagnostics and refine calibration.";
      if (tab === "fit") render();
      else toast(jobMessage);
    } else {
      if (JSON.stringify(requestedScenario) !== JSON.stringify(scenario)) {
        toast("Scenario changed; run the shot search again.");
        return;
      }
      shot = data.result.shot;
      candidates = data.result.candidates;
      if (!shot) {
        toast(
          "No robust scoring candidate found. Inspect calibration, geometry and the feasible region.",
        );
        return;
      }
      angle = shot.angle;
      speed = shot.speed;
      yaw = shot.yaw;
      spin = spinAt(project, speed, angle);

      if (tab === "explore") render();
    }
  };
  worker.onerror = (e) => {
    worker?.terminate();
    worker = null;
    jobMessage = e.message;
    toast(e.message);
    if (tab === "fit") render();
  };
  worker.postMessage({ kind, project, scenario });
  if (tab === "fit") render();
}
function logs() {
  $("#content").innerHTML =
    `<div class="notice"><span>ADVANTAGEKIT / WPILOG</span><div><strong>Align robot state with observed shots.</strong><p>Mark the release timestamp in robot-log seconds. Measured wheel velocity stays separate from commanded RPS. Ball speed, elevation and spin must still be measured externally.</p></div></div><div class="two-up"><div class="panel">${title("01 / LOG FILE", "Import a robot session")}<label class="file drop">↑ Choose a .wpilog file<input id="log-file" type="file" accept=".wpilog"></label><p class="muted">Files are decoded on this laptop. Primitive channels plus Pose2d and ChassisSpeeds structs are supported.</p><div id="log-summary">${log ? `<strong>${log.channels.length} channels · ${log.start.toFixed(3)}–${log.end.toFixed(3)} s</strong>${log.warnings.map((w) => `<p class="warning">${escape(w)}</p>`).join("")}` : '<p class="empty">No robot log loaded</p>'}</div></div><div class="panel">${title("02 / SHOT EVENT", "Mark an observed release")}<div class="inputs">${number("mark-time", "Release timestamp · robot seconds", log?.start ?? 0, 0.001)}<label class="control">Turret<select id="mark-turret"><option>Left</option><option>Right</option></select></label><label class="control">Observed outcome<select id="mark-outcome"><option>score</option><option>miss</option><option>rim</option></select></label></div><button id="mark-shot" ${log ? "" : "disabled"}>Attach telemetry to a trial +</button><p class="muted">Uses the last recorded value at or before release (WPILOG records changes). Missing required mechanism channels are reported, never replaced with zero.</p></div></div><div class="panel">${title("03 / CHANNEL INVENTORY", "Recorded signals")}<div class="table-scroll"><table><thead><tr><th>Channel</th><th>Type</th><th>Samples</th><th>First / last time</th></tr></thead><tbody>${
      log?.channels
        .filter((c) => c.samples.length)
        .slice(0, 300)
        .map(
          (c) =>
            `<tr><td>${escape(c.name)}</td><td>${escape(c.type)}</td><td>${c.samples.length}</td><td>${c.samples[0].time.toFixed(3)} / ${c.samples.at(-1)!.time.toFixed(3)}</td></tr>`,
        )
        .join("") ??
      '<tr><td class="empty">Load a session to inspect its channels.</td></tr>'
    }</tbody></table></div></div>`;
  $("#log-file").onchange = async (e) => {
    try {
      const f = (e.target as HTMLInputElement).files?.[0];
      if (f) {
        log = readLog(await f.arrayBuffer());
        render();
        toast("Robot log loaded.");
      }
    } catch (e) {
      toast((e as Error).message);
    }
  };
  $("#mark-shot").onclick = () => {
    try {
      if (!log) return;
      const timestamp = read("mark-time"),
        turret = $<HTMLSelectElement>("#mark-turret").value,
        values = snapshot(log, timestamp),
        base = `ShotLab/${turret}/`;
      const get = (key: string) => {
        const entry = Object.entries(values).find(
          ([name]) =>
            name.replace(/^\/?(?:RealOutputs\/|ReplayOutputs\/)?/, "") ===
            base + key,
        );
        return entry?.[1];
      };
      if (get("Mode") === "Legacy ferry")
        throw Error(
          "This is a ferry event. Hub calibration accepts hub and manual-tuning trials.",
        );
      const hood = get("HoodCommandDeg"),
        rps = get("MeasuredWheelRps");
      if (typeof hood !== "number" || typeof rps !== "number")
        throw Error(
          `Missing ${base}HoodCommandDeg / MeasuredWheelRps. Use a log recorded with the Shot Lab telemetry integration.`,
        );
      const command = get("WheelCommandRps");
      const r: Trial = {
        id: crypto.randomUUID().slice(0, 8),
        turret,
        timestamp,
        hood,
        rps,
        commandRps: typeof command === "number" ? command : undefined,
        outcome: $<HTMLSelectElement>("#mark-outcome").value,
        source: "WPILOG + observed release",
        telemetry: values,
        notes:
          "Ball elevation, exit speed and spin are not measured by robot telemetry.",
      };
      for (const [field, key] of [
        ["distance", "DistanceM"],
        ["radial", "RadialMps"],
        ["tangential", "TangentialMps"],
        ["azimuth", "BearingRad"],
      ] as const) {
        const v = get(key);
        if (typeof v === "number") r[field] = v;
      }
      addTrials([r]);
      render();
      toast(
        "Telemetry trial saved. Export CSV to add externally measured ball data, then re-import.",
      );
    } catch (e) {
      toast((e as Error).message);
    }
  };
}
render();
if (!project.comparison.length)
  fetch("./comparison.json")
    .then((r) => r.json())
    .then((r) => {
      project.comparison = r.points;
      persist(false);
    })
    .catch(() => {});
