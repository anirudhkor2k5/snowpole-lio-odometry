"""Diagnose any pipeline input CSV (incremental_navigation_results.csv format).

  python lio_bench/verify/diagnose_nav_csv.py path/to/file.csv [more.csv ...]     (from the repo root)

For each CSV:
  1. how far the CSV has dead-reckoned (sum of its row-to-row steps, which is all the pipeline uses)
     when the car has really driven 0.2 / 0.5 / 0.72 (first pole) / 1.0 km;
  2. runs the professor's pipeline (validated re-implementation, original mode) and prints its in-box
     mean / median, plus the pole-span error split into along-track and cross-track parts.

A CSV that is more than ~9 m (half the 18 m pole spacing) ahead of or behind the truth at the first
pole snaps to the wrong pole along the road and stays locked there. A cross-track part of ~+10 m is
the wrong-side lock (poles stand in pairs 8.8 m apart across the road).
"""
import contextlib
import io
import sys
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "lio_bench" / "pipeline"))
import snowpole_stage3 as S  # noqa: E402

N_ROWS = 5422          # rows in the template; shorter files are padded by holding the last row


def cumlen(e, n):
    return np.r_[0, np.cumsum(np.hypot(np.diff(e), np.diff(n)))]


def main(paths):
    with tempfile.TemporaryDirectory() as tmp:
        S.OUT = Path(tmp)
        for k, p in enumerate(paths):
            d = pd.read_csv(p)
            print(f"\n{p}\n  rows {len(d)}, Time {d['Time'].iloc[0]:.3f} .. {d['Time'].iloc[-1]:.3f}")
            if len(d) < N_ROWS:
                d = pd.concat([d, d.iloc[[-1] * (N_ROWS - len(d))]])
            csv = Path(tmp) / f"in{k}.csv"
            d.to_csv(csv, index=False)
            with contextlib.redirect_stdout(io.StringIO()):
                r = S.run(str(csv), "none", f"d{k}")
            f = pd.read_csv(Path(tmp) / f"d{k}_frames.csv")
            s_true = f["s_km"].to_numpy()
            s_csv = cumlen(d["easting"].to_numpy(float), d["northing"].to_numpy(float)) / 1000
            print("  CSV distance when the car has really driven: " + ", ".join(
                f"{km} km -> {s_csv[np.searchsorted(s_true, km)]:.3f} km" for km in (0.2, 0.5, 0.72, 1.0)))
            # along / cross-track split of the pipeline error over the pole span (truth heading per frame)
            re, rn, pe, pn = (f[c].to_numpy() for c in ("ref_e", "ref_n", "pred_e", "pred_n"))
            h = np.arctan2(np.gradient(rn), np.gradient(re))
            de, dn = pe - re, pn - rn
            along, cross = de * np.cos(h) + dn * np.sin(h), -de * np.sin(h) + dn * np.cos(h)
            span = (s_true >= r["pole_span"]["from_km"]) & (s_true <= r["pole_span"]["to_km"])
            nb = r["native_box_metric"]
            print(f"  pipeline in-box mean {nb['mean']:.1f} m, median {nb['median']:.1f} m; pole span: "
                  f"along-track median {np.median(along[span]):+.1f} m, cross-track median "
                  f"{np.median(cross[span]):+.1f} m (+ = ahead / left); pole fixes {r['pole_fixes']}")


if __name__ == "__main__":
    main(sys.argv[1:] or [str(REPO / "lio_bench" / "results" / "export" / "incremental_navigation_faster_lio_full_raw.csv")])
