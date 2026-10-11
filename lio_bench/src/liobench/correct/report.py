"""Every correction strategy against its baseline, per chronological block.

Selection discipline (single drive, so this is what keeps the numbers honest):
  - all fitted quantities (wheel lag/scale/axis, clock residuals) come from the
    training block;
  - the strategy is chosen on the validation block only;
  - the test block is reported for the chosen strategy and for the baseline,
    never used to choose anything.
Within a block, trajectories are anchored on the block's own first 200 m
(se2_head), so block numbers measure drift inside the block, not errors
inherited from earlier blocks. '+poles' runs are absolute (UTM), so for them the
unaligned ('none') error is reported as well.
"""
from __future__ import annotations

import json

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from ..io import RESULTS_ROOT, load_config, load_ground_truth, load_run, results_dir, run_meta  # noqa: E402
from ..metrics import evaluate  # noqa: E402
from .common import split_intervals  # noqa: E402

SUPPORTS = ("dlio", "genz_lio", "kiss_icp")
SELECT_ON = "drift_pct"          # lower is better; KITTI-style mean drift over 100-800 m


def _metrics(tr, gt, protocol, ecfg) -> dict:
    s = evaluate(tr, gt, protocol, ecfg).summary
    return {"ate_rmse_m": s["ate_xy"].get("rmse"), "final_m": s["final_error_m"],
            "drift_pct": s["drift_pct_mean_kitti"], "drift_200m_pct": s["drift_pct"]["200m"].get("mean"),
            "rpe1s_median_m": s["rpe_time"]["1s"].get("median"), "rpe10s_median_m": s["rpe_time"]["10s"].get("median"),
            "length_ratio": s["path_length_m"]["ratio"], "coverage": s["coverage"]}


def candidates(baseline: str, tag: str = "full") -> list[str]:
    root = RESULTS_ROOT / "runs"
    mine = sorted(d.name for d in root.iterdir() if d.name.startswith(f"{baseline}+")
                  and (d / tag / "odom_sensor_clock.tum").exists()) if root.exists() else []
    sup = [s for s in SUPPORTS + tuple(f"{x}+planar" for x in SUPPORTS)
           if (root / s / tag / "odom_sensor_clock.tum").exists()]
    return [baseline] + mine + sup


def run(cfg: dict | None = None, baseline: str = "fast_lio", tag: str = "full") -> dict:
    cfg = cfg or load_config()
    ecfg = dict(cfg["evaluation"])
    gt = load_ground_truth(cfg)
    blocks = split_intervals(cfg)
    names = candidates(baseline, tag)
    rep = {"baseline": baseline, "blocks_unix": blocks, "select_on": SELECT_ON, "methods": {}}
    for name in names:
        tr = load_run(name, tag, cfg)
        if tr is None or len(tr) < 100:
            continue
        absolute = run_meta(name, tag).get("world_frame") == "utm"
        row = {"whole_drive": _metrics(tr, gt, "se2_head", ecfg), "ms_per_scan": run_meta(name, tag).get("ms_per_scan")}
        if absolute:
            row["whole_drive_unaligned"] = _metrics(tr, gt, "none", ecfg)
        for b, (lo, hi) in blocks.items():
            g = gt.slice_time(lo, hi)
            t = tr.slice_time(lo - 2.0, hi + 2.0)
            if len(t) < 50:
                continue
            row[b] = _metrics(t, g, "se2_head", ecfg)
            if absolute:
                row[f"{b}_unaligned"] = _metrics(t, g, "none", ecfg)
        rep["methods"][name] = row

    def score(n, b):
        v = rep["methods"].get(n, {}).get(b, {}).get(SELECT_ON)
        return np.inf if v is None or not np.isfinite(v) else v

    correctors = [n for n in rep["methods"] if n.startswith(f"{baseline}+")]
    if correctors:
        best = min(correctors, key=lambda n: score(n, "val"))
        rep["selected_on_val"] = best
        rep["test"] = {"baseline": rep["methods"][baseline].get("test"), "selected": rep["methods"][best].get("test"),
                       "best_on_test_for_reference_only": min(correctors, key=lambda n: score(n, "test"))}
    out = results_dir("correct", baseline)
    (out / "correct_summary.json").write_text(json.dumps(rep, indent=2, default=float))
    _markdown(rep, out)
    _plot(rep, out)
    return rep


def _f(x, nd=2):
    return "—" if x is None or not np.isfinite(x) else f"{x:.{nd}f}"


def _markdown(rep, out):
    b = rep["baseline"]
    L = [f"# Correction strategies on top of {b}\n",
         "Per block: anchored on the block's own first 200 m, then pure prediction. "
         "Drift % = KITTI-style mean over 100-800 m segments (frame-free). "
         "Fitted quantities use the training block only; the strategy is selected on validation.\n",
         "| method | ms/scan | train drift % | val drift % | test drift % | test RPE 1 s m | test length ratio | whole-drive ATE m (se2_head) |",
         "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for n, r in rep["methods"].items():
        L.append(f"| {n} | {_f(r.get('ms_per_scan'), 0)} | {_f(r.get('train', {}).get('drift_pct'))} | "
                 f"{_f(r.get('val', {}).get('drift_pct'))} | {_f(r.get('test', {}).get('drift_pct'))} | "
                 f"{_f(r.get('test', {}).get('rpe1s_median_m'), 3)} | {_f(r.get('test', {}).get('length_ratio'), 3)} | "
                 f"{_f(r['whole_drive'].get('ate_rmse_m'), 1)} |")
    absn = [n for n, r in rep["methods"].items() if "whole_drive_unaligned" in r]
    if absn:
        L.append("\n## Absolute (UTM, no re-alignment) for pole-corrected runs\n")
        L.append("| method | whole drive ATE m | train ATE m | val ATE m | test ATE m |")
        L.append("|---|---:|---:|---:|---:|")
        for n in absn:
            r = rep["methods"][n]
            L.append(f"| {n} | {_f(r['whole_drive_unaligned'].get('ate_rmse_m'), 1)} | "
                     + " | ".join(_f(r.get(f'{b2}_unaligned', {}).get('ate_rmse_m'), 1) for b2 in ("train", "val", "test")) + " |")
    if "selected_on_val" in rep:
        t = rep["test"]
        L.append(f"\n**Selected on validation:** `{rep['selected_on_val']}`. Test drift: "
                 f"{_f((t['selected'] or {}).get('drift_pct'))} % vs baseline {_f((t['baseline'] or {}).get('drift_pct'))} %. "
                 f"(Best on test, for reference only, not a selection: `{t['best_on_test_for_reference_only']}`.)")
    (out / "correct_summary.md").write_text("\n".join(L) + "\n")


def _plot(rep, out):
    names = list(rep["methods"])
    if not names:
        return
    fig, ax = plt.subplots(figsize=(max(6, 0.9 * len(names)), 4))
    w = 0.27
    for i, b in enumerate(("train", "val", "test")):
        v = [rep["methods"][n].get(b, {}).get("drift_pct", np.nan) or np.nan for n in names]
        ax.bar(np.arange(len(names)) + (i - 1) * w, v, w, label=b)
    ax.set_xticks(np.arange(len(names)), names, rotation=35, ha="right", fontsize=8)
    ax.set_ylabel("drift % (KITTI mean)")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "drift_by_block.png", dpi=130)
    plt.close(fig)
