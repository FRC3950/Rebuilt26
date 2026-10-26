"""Render a sweep CSV as hit/miss and robustness maps (rows = distance, cols = bearing)."""
import csv
import sys
from collections import defaultdict

path = sys.argv[1]
step = float(sys.argv[2]) if len(sys.argv) > 2 else 0.1
rows = list(csv.DictReader(open(path)))
grid = {(round(float(r["d"]), 3), int(float(r["bearing"]))): r for r in rows}
ds = sorted({k[0] for k in grid})
bs = list(range(-90, 91, 5))
show = [d for d in ds if abs((d - 1.0) / step - round((d - 1.0) / step)) < 1e-6]

def cell(r, kind):
    if r is None:
        return " "
    if kind == "hit":
        if r["scored"] == "1":
            return "b" if r["bounced"] == "1" else "#"
        a = float(r["crossAlong"])
        if a != a:  # NaN: never came down through the entry height
            return "n"
        return "L" if a > 0 else "S"
    p = int(r["pert27"])
    return "x" if r["scored"] != "1" else ("9" if p == 27 else str(min(8, p // 3)))

for kind, title in (("hit", "HIT/MISS"), ("pert", "ROBUSTNESS")):
    print(f"\n{title}  rows: table distance d (m), cols: bearing deg (-90 .. +90, 0 = straight back toward blue wall)")
    print("      " + "".join("|" if b % 30 == 0 else " " for b in bs))
    for d in show:
        print(f"{d:5.2f} " + "".join(cell(grid.get((d, b)), kind) for b in bs))

n = len(rows)
hits = sum(r["scored"] == "1" for r in rows)
print(f"\npositions {n}, scored {hits}, missed {n - hits}")
band = defaultdict(lambda: [0, 0])
for r in rows:
    d = float(r["d"])
    key = "d<1.3 (below table)" if d < 1.3 - 1e-9 else f"{int(d * 2) / 2:.1f}-{int(d * 2) / 2 + 0.5:.1f}"
    band[key][0] += 1
    band[key][1] += r["scored"] == "1"
for k in sorted(band):
    t, h = band[k]
    print(f"  {k:22s} {h:4d}/{t:<4d} scored")
