"""One file with every result: results/REPORT.md (liobench summary).

Collects the per-step summaries the pipeline writes, plus a table of every run
(two sanity numbers each) and the pipeline's step status, so the whole study can
be read without anyone at the terminal.
"""
from __future__ import annotations

import json
import time

from .io import RESULTS_ROOT, load_config, load_ground_truth, load_run
from .metrics import evaluate


def _runs_table(cfg) -> list[str]:
    gt = load_ground_truth(cfg)
    rows = ["| run | tag | duration s | ATE se2_full m | length ratio | drift % | ms/scan |",
            "|---|---|---:|---:|---:|---:|---:|"]
    root = RESULTS_ROOT / "runs"
    for algo_dir in sorted(p for p in root.iterdir() if p.is_dir() and not p.name.startswith("_")):
        for tag_dir in sorted(p for p in algo_dir.iterdir() if (p / "odom_sensor_clock.tum").exists()):
            try:
                tr = load_run(algo_dir.name, tag_dir.name, cfg)
                if tr is None or len(tr) < 50:
                    continue
                g = gt.slice_time(tr.t[0] + 1.0, tr.t[-1] - 1.0)
                s = evaluate(tr, g, "se2_full", cfg["evaluation"]).summary
                meta = json.loads((tag_dir / "meta.json").read_text()) if (tag_dir / "meta.json").exists() else {}
                ms = meta.get("ms_per_scan")
                rows.append(f"| {algo_dir.name} | {tag_dir.name} | {tr.t[-1] - tr.t[0]:.0f} | "
                            f"{s['ate_xy'].get('rmse', float('nan')):.1f} | {s['path_length_m']['ratio']:.3f} | "
                            f"{s['drift_pct_mean_kitti']:.2f} | {'' if ms is None else f'{ms:.0f}'} |")
            except Exception as exc:  # noqa: BLE001 - one bad run must not hide the rest
                rows.append(f"| {algo_dir.name} | {tag_dir.name} | error: {exc} | | | | |")
    return rows


def _steps() -> list[str]:
    done = sorted(p.name for p in (RESULTS_ROOT / ".done").glob("*")) if (RESULTS_ROOT / ".done").exists() else []
    logs = sorted(p.stem for p in (RESULTS_ROOT / "logs").glob("*.log")) if (RESULTS_ROOT / "logs").exists() else []
    failed = [s for s in logs if s not in done]
    L = [f"- finished steps ({len(done)}): {', '.join(done)}"]
    L.append(f"- **not finished / failed** ({len(failed)}): {', '.join(failed) or 'none'}"
             + ("  — see results/logs/<step>.log" if failed else ""))
    return L


def _concurrency() -> list[str]:
    L = []
    for f in sorted((RESULTS_ROOT / "concurrency").glob("*.json")):
        r = json.loads(f.read_text())
        L.append(f"\n**{f.stem}**\n")
        L.append("| method | run | ATE m | drift % | latency median / p95 ms | scans w/o pose | CPU % | RAM MB |")
        L.append("|---|---|---:|---:|---|---:|---:|---:|")
        for a, row in r["per_algo"].items():
            for k in ("solo", "concurrent"):
                v = row.get(k)
                if not isinstance(v, dict):
                    L.append(f"| {a} | {k} | {v} | | | | | |")
                    continue
                lat = (f"{v['latency_ms_median']:.0f} / {v['latency_ms_p95']:.0f}" if "latency_ms_median" in v else "n/a")
                L.append(f"| {a} | {k} | {v.get('ate_rmse_m', float('nan')):.1f} | {v.get('drift_pct', float('nan')):.2f} | "
                         f"{lat} | {v.get('scans_without_pose', '')} | {v.get('cpu_pct_mean', float('nan')):.0f} | "
                         f"{v.get('rss_mb_max', float('nan')):.0f} |")
    return L


def run(cfg: dict | None = None) -> str:
    cfg = cfg or load_config()
    L = [f"# lio_bench — full report\n\nGenerated {time.strftime('%Y-%m-%d %H:%M')}. "
         "Every section below is also a separate file under results/ (paths given).\n",
         "## Pipeline status\n", *_steps(),
         "\n## Every run (whole run span, rigid planar fit)\n", *_runs_table(cfg)]
    sections = [("Step 1 — verification of fastlio.csv / fasterlio.csv", "verify/verify_summary.md"),
                ("Step 3 — weakness analysis", "analysis/analysis_summary.md")]
    for title, rel in sections:
        f = RESULTS_ROOT / rel
        L.append(f"\n---\n\n## {title}  (`results/{rel}`)\n")
        L.append(f.read_text() if f.exists() else "_not produced (step failed or not run)_")
    L.append("\n---\n\n## Baseline + support running at the same time  (`results/concurrency/`)\n")
    L += _concurrency() or ["_not produced_"]
    for group, pattern in (("Step 4 — learned fusion", "fusion/*/fusion_summary.md"),
                           ("Phase 2 — correction strategies", "correct/*/correct_summary.md"),
                           ("Step 5 — support in sliding windows", "windows/*/*/windows_summary.md")):
        L.append(f"\n---\n\n## {group}\n")
        files = sorted(RESULTS_ROOT.glob(pattern))
        if not files:
            L.append("_not produced_")
        for f in files:
            L.append(f"\n### `results/{f.relative_to(RESULTS_ROOT)}`\n")
            L.append(f.read_text())
    out = RESULTS_ROOT / "REPORT.md"
    out.write_text("\n".join(L) + "\n")
    return str(out)
