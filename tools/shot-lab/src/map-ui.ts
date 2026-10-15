import {
  baseline,
  compensate,
  evaluate,
  flight,
  type MapConfig,
  type RadialMap,
  type Band,
  type MapShot,
} from "./precomputed";
import "./map.css";
let config: MapConfig = { ...baseline },
  model: RadialMap | undefined,
  worker: Worker | undefined,
  bands: Band[] = [],
  best: MapShot | null = null;
try {
  const saved = JSON.parse(
    localStorage.getItem("chuck-radial-map-v2") ?? "null",
  );
  if (saved) {
    config = saved.config;
    model = saved.model;
  }
} catch {}
const app = document.querySelector<HTMLDivElement>("#app")!;
app.innerHTML = `<main><header><a href="/">CHUCK NORRIS / SHOT LAB</a><a href="make-miss-procedure.html">Simple make/miss procedure ↗</a><span>LOCAL · OFFLINE · V2</span></header><section class="hero"><p class="eyebrow">4414 STYLE TRAJECTORY MAPPING</p><h1>Search offline.<br><em>Aim with six terms.</em></h1><p>Sweep the scoring region. Pick the shot with the strongest error margin. Fit distance and radial velocity; cancel sideways motion with a 3D launch vector.</p></section><div class="notice">Baseline geometry and ideal transfer are estimates. Neutral drag and lift mean disabled corrections. Simulated scores do not establish real-world accuracy.</div><section class="layout"><div class="panel"><h2>01 / Scoring window</h2><p class="muted">All sampled angle/speed pairs are checked. Disconnected scoring bands stay separate. Red = lower speed boundary. Green = upper boundary.</p><div class="controls"><label>Distance · m<input id="distance" type="number" value="3.5" step=".1"></label><label>Toward hub · m/s<input id="radial" type="number" value="0" step=".1"></label><label>Across hub · m/s<input id="tangential" type="number" value="0" step=".1"></label></div><canvas id="window" width="900" height="430" aria-label="Scoring speed boundaries by launch elevation"></canvas><div class="legend"><span class="red">● Lower limit</span><span class="green">● Upper limit</span><span class="gold">◆ Robust shot</span></div><button id="sweep">Sweep scoring region</button><p id="shot">Choose a distance and radial velocity, then sweep.</p></div><div class="panel"><h2>02 / Physical inputs</h2><p class="muted">Copy measured dashboard values here. Changing map inputs requires regenerating the map. Delay and per-turret trims stay on the robot.</p><div id="inputs"></div><button id="save-config" class="secondary">Download inputs JSON</button><label class="file">Open inputs / map JSON<input id="open" type="file" accept=".json"></label></div></section><section class="panel"><h2>03 / Precompute the robot map</h2><p>Normalized quadratic terms: <code>1, d, r, d², dr, r²</code>. Seven outputs: elevation, exit speed, flight time and four asymmetric margins. Each patch must pass scoring re-simulation and a fit error below 10% of its reference margins. Variability inputs below are assumed until you measure repeated releases.</p><div class="actions"><button id="generate">Generate distance × radial map</button><button id="cancel" class="secondary" disabled>Cancel</button><button id="download" class="secondary">Download robot model</button></div><p id="status" role="status">No model generated in this browser yet.</p><canvas id="coverage" width="1000" height="260" aria-label="Accepted quadratic patches and unresolved coverage gaps"></canvas><p class="muted">Domain: 0.65–7.5 m, radial ±4.25 m/s (translation plus turret rotation). Green patches passed radial fit and trajectory checks; coral regions remain failures. Compensated mechanism-limit failures are counted separately below and rejected at runtime. The robot refuses uncovered inputs. Intermediate points and tangential compensation are re-simulated, but this is not a physical validation report.</p></section><section class="panel"><h2>Alliance motion coverage</h2><canvas id="field-coverage" width="460" height="600" aria-label="Alliance zone simulation success fraction by position"></canvas><p id="field-status" class="muted">Generate a map to test both turret offsets, four headings, stationary/1.25/2.5 m/s motion and ±maximum chassis rotation.</p><p class="muted">Each cell shows the fraction of tested states that scored within mechanism limits: green = all, gold = partial, coral = none. Flat chassis, neutral side trims and zero release delay. Tilt is compensated at runtime and checked separately by vector tests; hardware shots remain pending.</p></section><section class="panel"><h2>04 / Tune → generate → verify</h2><ol><li>Use <code>Tune/Use Tune Values</code> to measure the two endpoint launch elevations and wheel-to-ball transfer. <a href="/#start">Open the measurement instructions</a>.</li><li>Enter those values here, then measure repeated launch variation for angle and speed. Unknown drag, lift and spin stay zero until measured.</li><li>Generate the map on this laptop. Keep the app open; Cancel stops the worker immediately. Download <code>chuck_shot_model.json</code>.</li><li>Copy it to <code>Rebuilt26/src/main/deploy/chuck_shot_model.json</code>. Use your normal robot deployment procedure and restart the program. This app does not deploy anything.</li><li>Set the dashboard calibration to exactly match these inputs. A mismatch invalidates hub targeting. Delay, left/right hood, speed and yaw trims remain live; they change the requested shot and need physical retesting.</li><li>Test stationary near/middle/far shots, both turrets, then toward/away/across motion at increasing speeds up to 2.5 m/s, rotation, and representative pitch/roll. Record misses and refine one parameter at a time.</li></ol><p>At runtime the polynomial includes radial motion. The robot subtracts tangential velocity from the horizontal launch vector, recomputes elevation and speed from its 3D length, then transforms for chassis tilt and mechanism calibration. It includes each turret’s rotational velocity and release delay. Hood/RPS protections still apply.</p><p class="muted">The legacy <a href="advanced.html">measurement and log-import workspace</a> remains available. Its older version-1 layered model exports are incompatible with this radial version-2 solver.</p></section><footer>No push or deployment · Ferry and feed behavior unchanged · Robot.java unchanged</footer></main>`;
const $ = <T extends HTMLElement>(id: string) =>
  document.getElementById(id) as T;
const labels: Record<keyof MapConfig, string> = {
  height: "Release height · m",
  radius: "Wheel radius · m",
  angleMin: "Elevation at hood 13°",
  angleMax: "Elevation at hood 29.85°",
  transfer: "Ball speed transfer",
  drag: "Drag Cd",
  spinRatio: "Ball spin / wheel rev",
  lift: "Magnus coefficient",
  spinDecay: "Spin decay / s",
  sigmaAngle: "Measured angle variation · °",
  sigmaSpeed: "Measured speed variation · m/s",
};
for (const key of Object.keys(baseline) as (keyof MapConfig)[]) {
  const label = document.createElement("label");
  label.textContent = labels[key];
  const input = document.createElement("input");
  input.type = "number";
  input.step = "any";
  input.value = String(config[key]);
  input.oninput = () => {
    worker?.terminate();
    worker = undefined;
    $("cancel").setAttribute("disabled", "");
    config[key] = Number(input.value);
    model = undefined;
    bands = [];
    best = null;
    renderWindow();
    persist();
    renderCoverage();
  };
  label.append(input);
  $("inputs").append(label);
}
function persist() {
  try {
    localStorage.setItem(
      "chuck-radial-map-v2",
      JSON.stringify({ config, model }),
    );
  } catch {
    $("status").textContent =
      "Browser storage full; download JSON to keep this project.";
  }
}
function download(name: string, value: unknown) {
  const url = URL.createObjectURL(
    new Blob([JSON.stringify(value, null, 2)], { type: "application/json" }),
  );
  const a = document.createElement("a");
  a.href = url;
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
function run(kind: string) {
  worker?.terminate();
  worker = new Worker(new URL("./map-worker.ts", import.meta.url), {
    type: "module",
  });
  $("cancel").removeAttribute("disabled");
  $("status").textContent =
    kind === "generate"
      ? "Sweeping and fitting on this laptop…"
      : "Enumerating scoring region…";
  const activeWorker = worker;
  worker.onmessage = (e) => {
    if (worker !== activeWorker) return;
    if (e.data.kind === "progress") {
      $("status").textContent = e.data.text;
      return;
    }
    if (e.data.kind === "error") {
      $("status").textContent = e.data.text;
    } else if (kind === "sweep") {
      bands = e.data.result.bands;
      best = e.data.result.best;
      renderWindow();
      $("status").textContent =
        `${bands.length} sampled scoring bands. Resolution: ≤0.5° and ≤0.15 m/s, with refined speed boundaries.`;
    } else {
      model = e.data.result;
      persist();
      renderCoverage();
    }
    worker?.terminate();
    worker = undefined;
    $("cancel").setAttribute("disabled", "");
  };
  worker.onerror = (e) => {
    if (worker !== activeWorker) return;
    $("status").textContent = e.message;
    worker?.terminate();
    worker = undefined;
    $("cancel").setAttribute("disabled", "");
  };
  worker.postMessage({
    kind,
    config,
    distance: Number($<HTMLInputElement>("distance").value),
    radial: Number($<HTMLInputElement>("radial").value),
  });
}
for (const id of ["distance", "radial", "tangential"]) {
  $<HTMLInputElement>(id).oninput = () => {
    worker?.terminate();
    worker = undefined;
    $("cancel").setAttribute("disabled", "");
    bands = [];
    best = null;
    renderWindow();
    $("shot").textContent = "Scenario changed; run the scoring sweep again.";
  };
}
$("sweep").onclick = () => run("sweep");
$("generate").onclick = () => run("generate");
$("cancel").onclick = () => {
  worker?.terminate();
  worker = undefined;
  $("cancel").setAttribute("disabled", "");
  $("status").textContent = "Cancelled. Previous completed map retained.";
};
$("save-config").onclick = () => download("chuck-map-inputs.json", config);
$("download").onclick = () => {
  if (!model) {
    $("status").textContent = "Generate a map first.";
    return;
  }
  download("chuck_shot_model.json", model);
};
$<HTMLInputElement>("open").onchange = async (e) => {
  try {
    const file = (e.target as HTMLInputElement).files?.[0];
    if (!file) return;
    const value = JSON.parse(await file.text());
    const next = value.config ?? value;
    if (!Object.keys(baseline).every((k) => Number.isFinite(next[k])))
      throw Error("Missing physical input");
    if (
      value.config &&
      (value.version !== 2 || value.kind !== "radial-quadratic")
    )
      throw Error("Incompatible model");
    config = next;
    model = value.config ? value : undefined;
    persist();
    location.reload();
  } catch (e) {
    $("status").textContent = String(e);
  }
};
function renderWindow() {
  const c = $<HTMLCanvasElement>("window"),
    x = c.getContext("2d")!;
  x.clearRect(0, 0, c.width, c.height);
  const amin = Math.min(config.angleMin, config.angleMax),
    amax = Math.max(config.angleMin, config.angleMax),
    max = Math.max(15, ...bands.map((b) => b.upper)) * 1.08;
  const px = (a: number) => 65 + ((a - amin) / (amax - amin)) * 790,
    py = (v: number) => 370 - (v / max) * 325;
  x.strokeStyle = "#385257";
  x.beginPath();
  x.moveTo(65, 30);
  x.lineTo(65, 370);
  x.lineTo(860, 370);
  x.stroke();
  x.fillStyle = "#9fb5b5";
  x.font = "14px sans-serif";
  x.fillText("Exit speed · m/s", 65, 22);
  x.fillText("Launch elevation · degrees", 340, 417);
  for (let i = 0; i <= 4; i++) {
    x.fillText(((max * i) / 4).toFixed(1), 12, py((max * i) / 4) + 5);
    const a = amin + ((amax - amin) * i) / 4;
    x.fillText(a.toFixed(1), px(a) - 12, 395);
  }
  for (const b of bands) {
    x.strokeStyle = "#78bb9644";
    x.lineWidth = 4;
    x.beginPath();
    x.moveTo(px(b.angle), py(b.lower));
    x.lineTo(px(b.angle), py(b.upper));
    x.stroke();
    for (const [v, color] of [
      [b.lower, "#f27d76"],
      [b.upper, "#a7ed89"],
    ] as const) {
      x.fillStyle = color;
      x.beginPath();
      x.arc(px(b.angle), py(v), 3, 0, Math.PI * 2);
      x.fill();
    }
  }
  if (best) {
    const [a, v, t, ...m] = best.values;
    x.fillStyle = "#f4d68c";
    x.beginPath();
    x.arc(px(a), py(v), 7, 0, Math.PI * 2);
    x.fill();
    const tangent = Number($<HTMLInputElement>("tangential").value),
      q = compensate(a, v, tangent),
      d = Number($<HTMLInputElement>("distance").value),
      r = Number($<HTMLInputElement>("radial").value),
      f = flight(config, d, r, a, v, tangent);
    $("shot").textContent =
      `Selected: ${a.toFixed(2)}° · ${v.toFixed(2)} m/s · ${t.toFixed(3)} s. Margins: −${m[0].toFixed(2)}/+${m[1].toFixed(2)}°; −${m[2].toFixed(2)}/+${m[3].toFixed(2)} m/s. After sideways compensation: ${q.angle.toFixed(2)}°, ${q.speed.toFixed(2)} m/s, yaw ${((q.yaw * 180) / Math.PI).toFixed(2)}°. Re-simulation: ${f.scored ? "score" : "MISS — unresolved"}.`;
  } else
    $("shot").textContent =
      "No scoring region at the sampled resolution. Check geometry, input mapping and motion.";
}
function renderCoverage() {
  const field = $<HTMLCanvasElement>("field-coverage"),
    ctx = field.getContext("2d")!;
  ctx.clearRect(0, 0, field.width, field.height);
  $("field-status").textContent =
    "Generate a map to validate the alliance motion grid.";
  if (model?.coverage) {
    const v = model.coverage;
    for (const p of v.points) {
      const f = p.passed / p.tested;
      ctx.fillStyle =
        f === 1
          ? "#a7ed89"
          : f === 0
            ? "#f27d76"
            : `hsl(${30 + 65 * f} 45% 56%)`;
      ctx.fillRect(50 + p.x * 60 - 13, 30 + (8.0756 - p.y) * 60 - 14, 26, 28);
    }
    ctx.fillStyle = "#dce7e4";
    ctx.font = "14px sans-serif";
    ctx.fillText("Alliance x: 0 → 5.18 m · y: 0 → 8.08 m", 50, 565);
    $("field-status").textContent =
      `${v.passed.toLocaleString()} / ${v.total.toLocaleString()} simulated states score. ${v.uncovered.toLocaleString()} uncovered fit inputs; ${v.mechanismLimitFailures.toLocaleString()} mechanism/envelope failures; ${v.simulatedMisses.toLocaleString()} simulated misses. Full coverage is ${v.passed === v.total ? "simulated only" : "NOT established"}. Physical validation pending.`;
  }

  const c = $<HTMLCanvasElement>("coverage"),
    x = c.getContext("2d")!;
  x.clearRect(0, 0, c.width, c.height);
  if (!model) {
    $("status").textContent =
      "Map absent or stale. Generate after changing inputs.";
    return;
  }
  const [d0, d1, r0, r1] = model.domain;
  const draw = (b: number[], color: string) => {
    x.fillStyle = color;
    x.fillRect(
      48 + ((b[0] - d0) / (d1 - d0)) * 930,
      20 + ((r1 - b[3]) / (r1 - r0)) * 200,
      ((b[1] - b[0]) / (d1 - d0)) * 930,
      ((b[3] - b[2]) / (r1 - r0)) * 200,
    );
  };
  model.gaps.forEach((b) => draw(b, "#b85d59"));
  model.patches.forEach((p) => draw(p.bounds, "#81bc80"));
  x.fillStyle = "#bacbca";
  x.font = "14px sans-serif";
  x.fillText("Distance 0.65 → 7.5 m", 390, 248);
  x.fillText("+4.25", 0, 30);
  x.fillText("−4.25", 0, 218);
  $("status").textContent =
    `${model.patches.length} accepted patches · ${model.gaps.length} unresolved regions · ${model.validation.samples} fitted trajectory checks · maximum accepted fit error ${(100 * model.validation.maxErrorFraction).toFixed(2)}% of reference margins. ${model.validation.motionLimitFailures ?? 0} sampled compensated mechanism-limit failures (includes subdivided trial patches). Physical validation pending.`;
}
renderWindow();
renderCoverage();
if (!model)
  fetch("./baseline-map.json")
    .then((r) => (r.ok ? r.json() : null))
    .then((m) => {
      if (m && !model && JSON.stringify(config) === JSON.stringify(baseline)) {
        model = m;
        renderCoverage();
      }
    })
    .catch(() => {});
