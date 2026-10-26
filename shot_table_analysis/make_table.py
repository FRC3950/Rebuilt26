"""Write a candidate shot table: original rows with overrides. Usage: make_table.py out.json d=hood,rps[,tof] ..."""
import json, sys
src = json.load(open("shot_table_analysis/shot_table.original.json"))
rows = {r["d"]: dict(r) for r in src["points"]}
for arg in sys.argv[2:]:
    d, vals = arg.split("=")
    v = [float(x) for x in vals.split(",")]
    d = float(d)
    row = rows.get(d, {"d": d, "tof": None})
    row["hoodDeg"], row["rps"] = v[0], v[1]
    if len(v) > 2:
        row["tof"] = v[2]
    if row["tof"] is None:  # new row: placeholder tof, replaced by the measured value later
        row["tof"] = 1.0
    rows[d] = row
pts = [rows[k] for k in sorted(rows)]
for a, b in zip(pts, pts[1:]):
    assert b["hoodDeg"] >= a["hoodDeg"] and b["rps"] >= a["rps"], f"not monotonic at {a['d']}->{b['d']}"
json.dump({"units": src["units"], "points": pts}, open(sys.argv[1], "w"), indent=2)
