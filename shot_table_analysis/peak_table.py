"""Build the "peak" shot table from search_peak.csv.

Rules: every row scores at every in-region bearing and 27/27 perturbed shots; the arc apex is
held at one height H wherever the hood range allows it (closer in it rises as high as it can,
hood 13 being the steepest); hood and rps never decrease with distance; within those, the shot
whose crossing point is most centered in the opening wins.
Usage: peak_table.py H out.json
"""
import csv, collections, json, sys

H = float(sys.argv[1])
APEX_TOL = 0.08
CLOSE_MARGIN = 0.10  # stay this far under the highest robust apex where H is out of reach
ROWS = [1.55, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0, 3.25, 3.5, 3.75, 4.0, 4.25, 4.5, 4.75, 5.0, 5.25, 5.5, 5.75]

by = collections.defaultdict(list)
for r in csv.DictReader(open("shot_table_analysis/search_peak.csv")):
    if r["bearingsScored"] == r["bearingsTotal"] and r["pert27"] == "27":
        by[round(float(r["d"]), 3)].append(r)

layers = []
for d in ROWS:
    rs = by[d]
    target = min(H, max(float(r["apex"]) for r in rs) - CLOSE_MARGIN)
    cands = [r for r in rs if abs(float(r["apex"]) - target) <= APEX_TOL]
    if not cands:
        sys.exit(f"no candidate at d={d} for apex {target:.2f}")
    layers.append([(float(r["hood"]), float(r["rps"]),
                    abs(float(r["apex"]) - target) / 0.04 + abs(float(r["crossAlong"])) / 0.05, r)
                   for r in cands])

# DP: minimize cost with hood and rps non-decreasing; small penalty on jumps keeps rows smooth.
best = [{i: (c[2], None) for i, c in enumerate(layers[0])}]
for k in range(1, len(layers)):
    cur = {}
    for j, (h, p, cost, _) in enumerate(layers[k]):
        opts = [(v[0] + cost + 0.5 * (h - layers[k - 1][i][0]) + 0.2 * (p - layers[k - 1][i][1]), i)
                for i, v in best[k - 1].items()
                if layers[k - 1][i][0] <= h and layers[k - 1][i][1] <= p]
        if opts:
            cur[j] = min(opts)
    if not cur:
        sys.exit(f"monotonic chain breaks at d={ROWS[k]}")
    best.append(cur)
j = min(best[-1], key=lambda i: best[-1][i][0])
chosen = []
for k in range(len(layers) - 1, -1, -1):
    chosen.append(layers[k][j])
    j = best[k][j][1]
chosen.reverse()

points = [{"d": 1.3, "hoodDeg": 13.0, "rps": 43.0, "tof": 0.911}]  # close-range row, see README
for d, (h, p, _, r) in zip(ROWS, chosen):
    points.append({"d": d, "hoodDeg": h, "rps": p, "tof": round(float(r["tof"]), 3)})
    print(f"{d:4.2f} hood {h:5.2f} rps {p:6.2f} apex {float(r['apex']):.2f} tof {float(r['tof']):.3f} cross {float(r['crossAlong']):+.3f}")
json.dump({"units": {"distance": "m", "hood": "deg", "flywheel": "rps", "tof": "s"}, "points": points},
          open(sys.argv[2], "w"), indent=2)
