import { generateModel, solveShot, type Candidate } from "./solver";
import type { Project, Scenario } from "./types";
self.onmessage = async ({
  data,
}: {
  data: { kind: string; project: Project; scenario: Scenario };
}) => {
  try {
    const candidates: Candidate[] = [];
    const result =
      data.kind === "generate"
        ? await generateModel(data.project, (message, fraction) =>
            self.postMessage({ kind: "progress", message, fraction }),
          )
        : {
            shot: solveShot(data.project, data.scenario, candidates),
            candidates,
          };
    self.postMessage({ kind: "result", result });
  } catch (e) {
    self.postMessage({ kind: "error", message: (e as Error).message });
  }
};
