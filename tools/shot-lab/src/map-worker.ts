import { generate, sweep } from "./precomputed";
import { validateAlliance } from "./map-coverage";
self.onmessage = async (e) => {
  try {
    const { kind, config, distance, radial } = e.data;
    if (kind === "sweep")
      postMessage({ kind, result: sweep(config, distance, radial) });
    else {
      const result = await generate(config, undefined, (s) =>
        postMessage({ kind: "progress", text: s }),
      );
      postMessage({
        kind: "progress",
        text: "Validating alliance motion coverage…",
      });
      result.coverage = validateAlliance(result);
      postMessage({ kind, result });
    }
  } catch (error) {
    postMessage({ kind: "error", text: String(error) });
  }
};
