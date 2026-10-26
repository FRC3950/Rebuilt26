"""Compare two sweep CSVs: fixed, still scoring, regressions, still missing."""
import csv, sys
a = {(r["d"], r["bearing"]): r for r in csv.DictReader(open(sys.argv[1]))}
b = {(r["d"], r["bearing"]): r for r in csv.DictReader(open(sys.argv[2]))}
fixed = [k for k in a if a[k]["scored"] == "0" and b[k]["scored"] == "1"]
kept = [k for k in a if a[k]["scored"] == "1" and b[k]["scored"] == "1"]
reg = [k for k in a if a[k]["scored"] == "1" and b[k]["scored"] == "0"]
still = [k for k in a if a[k]["scored"] == "0" and b[k]["scored"] == "0"]
print(f"missed->scores {len(fixed)}, scored->scores {len(kept)}, REGRESSIONS {len(reg)}, still missing {len(still)}")
for k in reg:
    print("  regression d=%s bearing=%s pert %s->%s" % (k[0], k[1], a[k]["pert27"], b[k]["pert27"]))
worse = [k for k in kept if int(b[k]["pert27"]) < int(a[k]["pert27"])]
print(f"scored both times but lower perturbation count: {len(worse)}")
for k in sorted(worse, key=lambda k: float(k[0]))[:40]:
    print("  d=%s bearing=%s pert %s->%s" % (k[0], k[1], a[k]["pert27"], b[k]["pert27"]))
