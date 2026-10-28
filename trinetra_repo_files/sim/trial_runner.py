"""Run many randomised simulation trials and report the success rate.

Usage:
  python sim/trial_runner.py                  # 50 trials, 2 obstacles
  python sim/trial_runner.py --trials 100 --obstacles 3 --seed 7
Writes per-trial results to sim/results.csv.
"""
import argparse
import csv
import os
import sys
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import sim_core as sc  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trials", type=int, default=50)
    ap.add_argument("--obstacles", type=int, default=2)
    ap.add_argument("--seed", type=int, default=0, help="first seed; trial i uses seed+i")
    ap.add_argument("--out", default=os.path.join(HERE, "results.csv"))
    args = ap.parse_args()

    rows = []
    for i in range(args.trials):
        r = sc.run_episode(args.seed + i, args.obstacles)
        rows.append({k: r[k] for k in ("seed", "result", "time_s", "final_gap_cm")})

    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["seed", "result", "time_s", "final_gap_cm"])
        w.writeheader()
        w.writerows(rows)

    counts = Counter(r["result"] for r in rows)
    ok = [r for r in rows if r["result"] == "success"]
    print(f"Trials: {args.trials}   obstacles per trial: {args.obstacles}")
    print(f"Success: {len(ok)}/{args.trials} = {100 * len(ok) / args.trials:.1f}%")
    if ok:
        print(f"Mean time to reach target: {sum(r['time_s'] for r in ok) / len(ok):.1f} s")
    for k, v in counts.most_common():
        if k != "success":
            print(f"  {k}: {v}")
    print(f"Per-trial results saved to {args.out}")


if __name__ == "__main__":
    main()
