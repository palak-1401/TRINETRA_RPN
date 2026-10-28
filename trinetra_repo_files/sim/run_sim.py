"""Run the Trinetra robot simulation.

Usage:
  python sim/run_sim.py --scenarios            # 13 README test scenarios (PASS/FAIL)
  python sim/run_sim.py --seed 3               # one simulated run with a state trace
  python sim/run_sim.py --seed 3 --plot run.png  # also save the path as an image
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import robot_logic as rl  # noqa: E402
import sim_core as sc     # noqa: E402


def run_scenarios():
    """Check Controller against the 13 scenarios in README section 7.2."""
    def one(line, dist=400.0, c=None, t=0):
        c = c or rl.Controller()
        pkt = rl.parse_packet(line)
        return c, c.step(t, dist, pkt)

    results = []

    def check(n, name, ok):
        results.append(ok)
        print(f"  {n:>2}. {'PASS' if ok else 'FAIL'}  {name}")

    print("README test scenarios")
    _, (s, l, r) = one("V,0,0,0")
    check(1, "V,0,0,0 -> SEARCH rotates", s == "SEARCH" and l > 0 > r)
    _, (s, l, r) = one("V,1,-60,20")
    check(2, "V,1,-60,20 -> ALIGN turns left", s == "ALIGN" and l < 0 < r)
    _, (s, l, r) = one("V,1,60,20")
    check(3, "V,1,60,20 -> ALIGN turns right", s == "ALIGN" and l > 0 > r)
    _, (s, l4, r4) = one("V,1,0,20")
    check(4, "V,1,0,20 -> APPROACH forward", s == "APPROACH" and l4 > 0 and r4 > 0)
    _, (s, l5, r5) = one("V,1,0,60")
    check(5, "V,1,0,60 -> APPROACH slower", s == "APPROACH" and 0 < l5 < l4)
    _, (s, l, r) = one("V,1,0,90")
    check(6, "V,1,0,90 -> STOP", s == "STOP" and l == 0 and r == 0)
    c, _ = one("V,1,0,20")
    _, (s, _, _) = one("V,0,0,0", c=c, t=50)
    check(7, "target lost -> back to SEARCH", s == "SEARCH")
    _, (s, l, r) = one("V,1,0,20", dist=15)
    check(8, "obstacle 15 cm -> AVOID", s == "AVOID" and not (l > 0 and r > 0))
    _, (s, _, _) = one("V,1,0,20", dist=100)
    check(9, "obstacle 100 cm -> APPROACH", s == "APPROACH")
    _, (s, _, _) = one("V,0,0,0", dist=15)
    check(10, "AVOID overrides SEARCH", s == "AVOID")
    c, _ = one("V,1,0,20")
    s, l, r = c.step(1000, 400.0, None)
    check(11, "no input for 1 s -> failsafe stop", s == "FAILSAFE" and l == 0 and r == 0)
    check(12, "garbage ignored", rl.parse_packet("garbage") is None)
    c, _ = one("V,1,0,20", dist=15)
    s, _, _ = c.step(50, 400.0, rl.parse_packet("V,1,0,20"))
    check(13, "obstacle removed -> resumes APPROACH", s == "APPROACH")

    print(f"{sum(results)}/{len(results)} passed")
    return all(results)


def plot(world, out):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed: skipping plot (pip install matplotlib)")
        return
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(0, sc.ARENA)
    ax.set_ylim(0, sc.ARENA)
    ax.set_aspect("equal")
    for ox, oy, orad in world.obstacles:
        ax.add_patch(plt.Circle((ox, oy), orad, color="gray", label="obstacle"))
    ax.add_patch(plt.Circle((world.tx, world.ty), sc.TARGET_R, color="tab:green", label="target"))
    xs, ys = zip(*world.path)
    ax.plot(xs, ys, "-", color="tab:blue", label="robot path")
    ax.plot(xs[0], ys[0], "o", color="tab:blue")
    h, lab = ax.get_legend_handles_labels()
    uniq = dict(zip(lab, h))
    ax.legend(uniq.values(), uniq.keys(), loc="upper right", fontsize=8)
    ax.set_xlabel("cm")
    ax.set_ylabel("cm")
    fig.savefig(out, dpi=120, bbox_inches="tight")
    print(f"Saved path plot to {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenarios", action="store_true", help="run the 13 README test scenarios")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--obstacles", type=int, default=2)
    ap.add_argument("--plot", metavar="PNG", help="save the robot path to an image")
    args = ap.parse_args()

    if args.scenarios:
        sys.exit(0 if run_scenarios() else 1)

    last = {"state": None}

    def trace(w, state, dist, pkt):
        if state != last["state"]:
            print(f"t={w.t:5.2f}s  {state:<9} pos=({w.x:6.1f},{w.y:6.1f}) "
                  f"heading={math.degrees(w.th):6.1f}  dist={dist:5.1f}cm  "
                  f"pkt=V,{pkt.detected},{pkt.x_offset},{pkt.size}")
            last["state"] = state

    res = sc.run_episode(args.seed, args.obstacles, trace)
    print(f"\nResult: {res['result']} in {res['time_s']} s "
          f"(final gap to target {res['final_gap_cm']} cm)")
    if args.plot:
        plot(res["world"], args.plot)


if __name__ == "__main__":
    main()
