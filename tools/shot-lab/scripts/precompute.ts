import { readFileSync, writeFileSync } from "node:fs";
import { baseline, generate } from "../src/precomputed";
import { validateAlliance } from "../src/map-coverage";
const input = process.argv[2],
  output = process.argv[3] ?? "../../src/main/deploy/chuck_shot_model.json";
const config = input ? JSON.parse(readFileSync(input, "utf8")) : baseline;
let last = 0;
const model = await generate(config, undefined, (s) => {
  if (Date.now() - last > 10000) {
    console.log(s);
    last = Date.now();
  }
});
console.log("Validating alliance motion coverage...");
model.coverage = validateAlliance(model);
writeFileSync(output, JSON.stringify(model));
console.log(
  JSON.stringify({
    file: output,
    patches: model.patches.length,
    gaps: model.gaps.length,
    validation: model.validation,
  }),
);
