"""Recommend HFI_OMEGA_LOW from a shadow-sweep bench log.

Run the bench test with RUN_HFI 1 (shadow) and TEST_PROFILE 3 (slow sweep down), save the
serial output, then:

    python tools/hfi_threshold.py run.log [--pole-pairs 7] [--bin 25] [--factor 1.5]

While the HFI tracker runs alongside the eSMO (on = 1, ctl = 0), smo_hfi is the eSMO angle
minus the HFI angle. The tracker's noise does not depend on speed; the eSMO's grows as the
back-EMF shrinks. The script bins smo_hfi by the eSMO speed, takes the spread (sd) of the
fastest bins as the reference, and recommends the lowest speed above which every bin's sd
stays within --factor times that reference.
"""
import argparse
import math


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--pole-pairs", type=int, default=7)
    ap.add_argument("--bin", type=float, default=25.0, help="speed bin width (rad/s el)")
    ap.add_argument("--factor", type=float, default=1.5, help="allowed sd growth over the reference")
    ap.add_argument("--min-samples", type=int, default=8)
    a = ap.parse_args()

    cols = None
    bins = {}
    with open(a.log, encoding="ascii", errors="replace") as f:
        for line in f:
            p = line.split()
            if p[:2] == ["t_ms", "mode"]:
                cols = {name: i for i, name in enumerate(p)}
                continue
            if cols is None or len(p) != len(cols) or not p[0].lstrip("-").isdigit():
                continue
            v = {k: int(p[i]) for k, i in cols.items()}
            if v["mode"] != 8 or v["on"] != 1 or v["ctl"] != 0:
                continue
            b = int(abs(v["w_est"]) // a.bin)
            bins.setdefault(b, []).append(v["smo_hfi"] / 10.0)

    rows = []
    for b in sorted(bins):
        d = bins[b]
        if len(d) < a.min_samples:
            continue
        m = sum(d) / len(d)
        sd = math.sqrt(sum((x - m) ** 2 for x in d) / len(d))
        rows.append((b * a.bin, (b + 1) * a.bin, len(d), m, sd))
    if len(rows) < 3:
        raise SystemExit("not enough shadow samples (need RUN_HFI 1 with TEST_PROFILE 3)")

    ref_rows = rows[-2:]  # Fastest two bins
    ref = sum(r[4] for r in ref_rows) / len(ref_rows)
    print(f"{'rad/s el':>11} {'n':>4} {'mean':>7} {'sd':>6}")
    for lo, hi, n, m, sd in rows:
        print(f"{lo:5.0f}-{hi:<5.0f} {n:4d} {m:+6.1f} deg {sd:5.1f} deg{'  > limit' if sd > a.factor * ref else ''}")

    # Lowest bin edge with every bin from there up within factor * ref
    low = rows[-1][0]
    for lo, hi, n, m, sd in reversed(rows):
        if sd > a.factor * ref:
            break
        low = lo
    rpm = low * 60.0 / (2.0 * math.pi * a.pole_pairs)
    print(f"\nreference sd {ref:.1f} deg (fastest bins), limit {a.factor * ref:.1f} deg")
    print(f"recommended HFI_OMEGA_LOW = {low:.0f} rad/s el ({rpm:.0f} rpm mech at {a.pole_pairs} pole pairs)")


if __name__ == "__main__":
    main()
