"""How much of the after-last-pole result is the drift-rate extrapolation getting lucky?

Re-runs the stage-3 loop (reset+rate) with the learned rate shifted by a fixed offset, and with an
alternative but equally reasonable rate estimator (fit over only the most recent updates). Writes
results/pipeline/RATE_SENSITIVITY.md. Per-run frame CSVs go to a temp dir and are discarded.

  python lio_bench/pipeline/rate_sensitivity.py
"""
from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import snowpole_stage3 as S  # noqa: E402
from run_all_variants import INPUTS  # noqa: E402

LABELS = ["fastlio_raw", "fasterlio_raw", "fastlio_gk", "fasterlio_gk", "final_online"]
OFFSETS = [-0.010, -0.005, -0.002, 0.0, 0.002, 0.005, 0.010]       # deg/s added to the learned rate
WINDOWS = [None, 600.0, 300.0]                                      # s of update history in the rate fit

_orig_fix = S.HeadingCorrector.fix
CFG = {"offset": 0.0, "window": None}


def _fix(self, e, n, t, frame):
    _orig_fix(self, e, n, t, frame)
    if self.mode != "reset+rate" or len(self.updates) < S.RATE_MIN_UPDATES:
        return
    T = np.array(self.updates)
    if CFG["window"] is not None:
        T = T[T[:, 0] >= T[-1, 0] - CFG["window"]]
    if len(T) >= S.RATE_MIN_UPDATES and T[-1, 0] - T[0, 0] >= S.RATE_MIN_SPAN_S:
        self.base_rate = float(np.polyfit(T[:, 0] - T[0, 0], T[:, 1], 1)[0])
    if getattr(self, "base_rate", None) is not None:
        self.rate = self.base_rate + CFG["offset"]
    else:
        self.rate = 0.0


def main():
    S.HeadingCorrector.fix = _fix
    csv = {lab: str(p) for lab, p, _ in INPUTS}
    res = {}
    with tempfile.TemporaryDirectory() as tmp:
        S.OUT = Path(tmp)
        for lab in LABELS:
            for w in WINDOWS:
                for o in (OFFSETS if w is None else [0.0]):
                    CFG.update(offset=o, window=w)
                    r = S.run(csv[lab], "reset+rate", f"{lab}_{w}_{o}")
                    a = r["after_last_pole"]
                    res[(lab, w, o)] = (a["mean"], a["final_m"], r["whole_drive"]["mean"])
                    print(lab, w, o, f"after-pole mean {a['mean']:.1f} final {a['final_m']:.1f}", flush=True)
    L = ["# Drift-rate sensitivity of the after-last-pole result\n",
         "*`lio_bench/pipeline/rate_sensitivity.py`. Stage 3 = heading reset + drift rate. The learned rate is "
         "re-fitted at every accepted update; here it is shifted by a fixed offset, or fitted on only the "
         "last W seconds of updates. Cells: after-last-pole mean / final error [m] (≈5.1 km, no poles).*\n",
         "## Rate offset (full-history fit)\n",
         "| input | " + " | ".join(f"{o:+.3f}°/s" for o in OFFSETS) + " |",
         "|---|" + "---:|" * len(OFFSETS)]
    for lab in LABELS:
        L.append(f"| {lab} | " + " | ".join(f"{res[(lab, None, o)][0]:.0f} / {res[(lab, None, o)][1]:.0f}"
                                           for o in OFFSETS) + " |")
    L += ["\n## Rate-fit window (no offset)\n", "| input | all updates | last 600 s | last 300 s |",
          "|---|---:|---:|---:|"]
    for lab in LABELS:
        L.append(f"| {lab} | " + " | ".join(f"{res[(lab, w, 0.0)][0]:.0f} / {res[(lab, w, 0.0)][1]:.0f}"
                                           for w in WINDOWS) + " |")
    out = S.REPO / "lio_bench" / "results" / "pipeline"
    (out / "RATE_SENSITIVITY.md").write_text("\n".join(L) + "\n")
    (out / "rate_sensitivity.json").write_text(json.dumps({f"{k[0]}|{k[1]}|{k[2]}": v for k, v in res.items()}, indent=1))
    print("wrote", out / "RATE_SENSITIVITY.md")


if __name__ == "__main__":
    main()
