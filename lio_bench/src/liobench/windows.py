"""Step 5: run the supporting method only inside (adaptive) sliding windows.

Schedules (all causal; they only use information available at decision time):
  periodic   window of `length_s` every `period_s` (fixed duty cycle; the control)
  adaptive   open a window when the trigger model's predicted baseline error rises
             above a threshold (a quantile of its training-block predictions);
             keep it open while the prediction stays high, close it after
             `hold_s` below, clamp to [min_len_s, max_len_s]. Length adapts to
             how long the baseline is predicted to be unreliable.
Each window is run from `warmup_s` earlier (in deployment: a ring buffer of the
last few seconds of scans + IMU), cold (estimator assumes rest) or warm (filter
velocity seeded from the baseline). Only post-warm-up output is used.

Measured against the continuous support run and against the baseline:
  * support quality inside windows: step error windowed vs continuous (the drop)
  * transient: step error vs time since window start
  * fused result with the step-4 gating model, support available only in windows
  * duty cycle (share of time the support runs) and support compute time
"""
from __future__ import annotations

import json
import subprocess
from pathlib import Path

import numpy as np
import pandas as pd

from . import plots
from .dataset import build, _method_increments
from .io import BENCH_ROOT, load_config, load_run, results_dir, run_dir
from .metrics import estimate_time_offset
from .geometry import level


# --------------------------------------------------------------------------- schedules

def periodic(t: np.ndarray, length_s: float, period_s: float, t_from: float, t_to: float):
    starts = np.arange(t_from, t_to - length_s, period_s)
    return [(float(s), float(s + length_s)) for s in starts]


def adaptive(t: np.ndarray, score: np.ndarray, thr: float, t_from: float, t_to: float, hold_s=3.0,
             min_len_s=5.0, max_len_s=60.0):
    wins, open_at, below_since = [], None, None
    for ti, sc in zip(t, score):
        if ti < t_from or ti > t_to or not np.isfinite(sc):
            continue
        if open_at is None:
            if sc > thr:
                open_at, below_since = ti, None
        else:
            below_since = (below_since or ti) if sc <= thr else None
            too_long = ti - open_at >= max_len_s
            if too_long or (below_since is not None and ti - below_since >= hold_s and ti - open_at >= min_len_s):
                wins.append((float(open_at), float(ti)))
                open_at, below_since = None, None
    if open_at is not None:
        wins.append((float(open_at), float(min(t_to, open_at + max_len_s))))
    return wins


# --------------------------------------------------------------------------- running windows

def _bag_start_unix(cfg) -> float:
    from rosbags.highlevel import AnyReader
    from .io import repo_path
    with AnyReader([repo_path(cfg["reduced_bag"])]) as r:
        return r.start_time * 1e-9


def run_window(support: str, t0: float, t1: float, warmup_s: float, tag: str, cfg: dict, warm_from: str | None,
               bag_t0: float, force=False) -> Path:
    d = run_dir(support, tag)
    if (d / "odom_sensor_clock.tum").exists() and not force:
        return d
    start = max(0.0, t0 - warmup_s - bag_t0)
    dur = (t1 - t0) + warmup_s
    if support == "genz_lio":
        from . import genz_runner
        genz_runner.run(tag=tag, start_offset_s=start, duration_s=dur, warm_start_from=warm_from, progress=False)
    else:  # ROS estimators: real-time playback through the orchestrator (cold start only)
        subprocess.run(["bash", str(BENCH_ROOT / "scripts" / "run_ros.sh"), "--algos", support, "--start", f"{start:.3f}",
                        "--duration", f"{dur:.3f}", "--tag", tag, "--no-latency"], check=True)
    return d


def windowed_increments(support: str, wins, tags, st, warmup_s, cfg):
    """Support increments on the GT clock, available only inside each window after warm-up."""
    from .io import load_run
    n = len(st.t)
    u = np.full((n, 3), np.nan)
    since = np.full(n, np.nan)
    # The clock offset is a property of the sensor/estimator pair: reuse the continuous run's.
    full = load_run(support, "full", cfg)
    dt = estimate_time_offset(level(full), _gt(cfg))["dt_s"] if full is not None else 0.0
    for (a, b), tag in zip(wins, tags):
        tr = load_run(support, tag, cfg)
        if tr is None or len(tr) < 5:
            continue
        ui = _method_increments(tr.shifted(dt), st.t, cfg["evaluation"].get("max_assoc_dt", 0.15), cfg)
        rows = (st.t >= a) & (st.t <= b)
        # remove the training-block mounting yaw exactly as for the continuous run
        c, s = np.cos(-st.mount_yaw.get(support, 0.0)), np.sin(-st.mount_yaw.get(support, 0.0))
        f, l = ui[:, 0].copy(), ui[:, 1].copy()
        ui[:, 0], ui[:, 1] = c * f - s * l, s * f + c * l
        u[rows] = ui[rows]
        since[rows] = st.t[rows] - a
    return u, since


_GT = None


def _gt(cfg):
    global _GT
    if _GT is None:
        from .io import load_ground_truth
        _GT = load_ground_truth(cfg)
    return _GT


# --------------------------------------------------------------------------- experiment

def run(cfg: dict | None = None, baseline: str = "fast_lio", support: str = "genz_lio", schedule: str = "adaptive",
        warmup_s: float = 5.0, warm_start: bool = True, threshold_q: float = 0.75, length_s: float = 20.0,
        period_s: float = 60.0, hold_s: float = 3.0, min_len_s: float = 5.0, max_len_s: float = 60.0,
        only_test_block: bool = True, force: bool = False) -> dict:
    """Run one window schedule and compare against continuous support."""
    import torch
    from . import fusion

    cfg = cfg or load_config()
    name = f"{support}_{schedule}_{'warm' if warm_start else 'cold'}_wu{warmup_s:g}"
    if schedule == "adaptive":
        name += f"_q{threshold_q:g}"
    else:
        name += f"_L{length_s:g}_P{period_s:g}"
    out = results_dir("windows", baseline, name)
    fdir = results_dir("fusion", baseline)
    if not (fdir / "gating_model.pt").exists():
        raise RuntimeError("needs step 4 first: liobench fusion --baseline " + baseline)
    ck_methods = torch.load(fdir / "gating_model.pt", weights_only=False)["methods"]
    st = build(cfg, methods=list(dict.fromkeys(ck_methods + [support])))
    if support not in st.inc:
        raise RuntimeError(f"needs the continuous run of {support} first")
    trig = pd.read_csv(fdir / "trigger_predictions.csv")
    sp = cfg["split"]
    tr, va, te = fusion.chronological_split(len(st.t), st.t, sp["train"], sp["val"], sp["gap_s"])
    rows = te if only_test_block else np.arange(len(st.t))
    t_from, t_to = float(st.t[rows[0]]), float(st.t[rows[-1]])

    if schedule == "adaptive":
        thr = float(np.nanquantile(trig["pred_baseline_step_err"].to_numpy()[tr], threshold_q))
        wins = adaptive(st.t, trig["pred_baseline_step_err"].to_numpy(), thr, t_from, t_to, hold_s, min_len_s, max_len_s)
    else:
        thr = None
        wins = periodic(st.t, length_s, period_s, t_from, t_to)

    bag_t0 = _bag_start_unix(cfg)
    tags = [f"win_{baseline}_{name}_{k:03d}" for k in range(len(wins))]   # baseline in the name: windows differ per baseline
    for (a, b), tag in zip(wins, tags):
        run_window(support, a, b, warmup_s, tag, cfg, baseline if (warm_start and support == "genz_lio") else None,
                   bag_t0, force)

    u_win, since = windowed_increments(support, wins, tags, st, warmup_s, cfg)
    y = st.gt
    in_win = np.isfinite(u_win).all(1)
    blk = np.zeros(len(st.t), bool); blk[rows] = True

    def step_err(u, m):
        ok = m & np.isfinite(u).all(1) & np.isfinite(y).all(1)
        return np.linalg.norm(u[ok, :2] - y[ok, :2], axis=1), ok

    e_win, _ = step_err(u_win, in_win & blk)
    e_cont, _ = step_err(st.inc[support], in_win & blk)
    e_base, _ = step_err(st.inc[baseline], in_win & blk)

    # transient after window start
    tb = np.floor(np.nan_to_num(since, nan=-1)).astype(int)
    trans = []
    for s in range(0, int(np.nanmax(since)) + 1 if np.isfinite(since).any() else 0):
        m = (tb == s) & in_win & blk
        ew, _ = step_err(u_win, m)
        ec, _ = step_err(st.inc[support], m)
        if len(ew):
            trans.append({"s_since_start": s, "windowed": float(np.median(ew)), "continuous": float(np.median(ec))})

    # fused with the step-4 model, support only inside windows
    ck = torch.load(fdir / "gating_model.pt", weights_only=False)
    methods = ck["methods"]
    ctx_prefixes = fusion.CONTEXT_PREFIXES + (("genz_",) if "genz_lio" in methods[1:] else ())
    fit = None
    if ck.get("kind", "gating") == "gating":
        model = fusion.GatingMLP(len(ck["features"]), len(methods), ck.get("hidden", 64))
        model.load_state_dict(ck["state_dict"])
        sc = fusion.Standardizer(); sc.mu, sc.sd = ck["mu"], ck["sd"]
        fit = fusion.FitResult(model, sc, ck["features"], [])

    def fused(inc_override, ctx_off):
        U = np.stack([inc_override[m] for m in methods], 1)
        mask = np.isfinite(U).all(2)
        if fit is None:                     # CV chose static weights
            return fusion.apply_static(U, mask, np.asarray(ck["static_weights"]))
        X, _ = fusion.make_features(st, methods, ctx_prefixes, inc=inc_override, ctx_off=ctx_off)
        wt, wy = fusion.predict_weights(fit, X, mask)
        return np.c_[(wt[:, :, None] * np.nan_to_num(U[:, :, :2])).sum(1), (wy * np.nan_to_num(U[:, :, 2])).sum(1)]

    inc_cont = dict(st.inc)
    inc_win = dict(st.inc); inc_win[support] = u_win
    others_off = {m: np.full_like(st.inc[m], np.nan) for m in methods[1:] if m != support}
    inc_cont.update(others_off); inc_win.update(others_off)
    inc_base = dict(st.inc); inc_base.update({m: np.full_like(st.inc[m], np.nan) for m in methods[1:]})
    off_genz_all = {"genz_": np.ones(len(st.t), bool)}
    ctx_win = {"genz_": ~in_win} if support == "genz_lio" else off_genz_all
    ctx_cont = {} if support == "genz_lio" else off_genz_all
    res = {
        "baseline_only": fusion.evaluate_block("baseline_only", fused(inc_base, off_genz_all), st, rows),
        "fused_continuous_support": fusion.evaluate_block("fused_continuous", fused(inc_cont, ctx_cont), st, rows),
        "fused_windowed_support": fusion.evaluate_block("fused_windowed", fused(inc_win, ctx_win), st, rows),
    }
    duty = float(in_win[rows].mean())
    # support compute: wall time of the offline GenZ runner; CPU-seconds of the ROS node (from
    # its 0.5 s resource samples). ROS runs have no 'wall_s', which used to report 0 s for DLIO.
    cost = 0.0
    for tag in tags:
        d = run_dir(support, tag)
        mj, rc = d / "meta.json", d / "resources.csv"
        if rc.exists():
            r = pd.read_csv(rc)
            r = r[r["algo"] == support]
            cost += float((r["cpu_pct"] / 100.0).sum() * 0.5)
        elif mj.exists():
            cost += json.loads(mj.read_text()).get("wall_s", 0.0)
    report = {
        "baseline": baseline, "support": support, "schedule": schedule, "warm_start": warm_start, "warmup_s": warmup_s,
        "threshold": thr, "windows": [{"start": a - t_from, "end": b - t_from, "len": b - a} for a, b in wins],
        "n_windows": len(wins), "duty_cycle": duty, "support_wall_s": cost,
        "support_step_err_in_windows": {"windowed_median": float(np.median(e_win)) if len(e_win) else None,
                                        "continuous_median": float(np.median(e_cont)) if len(e_cont) else None,
                                        "baseline_median": float(np.median(e_base)) if len(e_base) else None},
        "transient": trans,
        "fused": {k: fusion._strip(v) for k, v in res.items()},
    }
    (out / "windows_summary.json").write_text(json.dumps(report, indent=2, default=float))
    if trans:
        tdf = pd.DataFrame(trans)
        plots.series(tdf["s_since_start"], {"windowed": tdf["windowed"], "continuous": tdf["continuous"]},
                     out / "transient_after_window_start.png", "seconds since window start",
                     "median step error [m]", f"{support}: start-up transient ({'warm' if warm_start else 'cold'})")
    plots.map_overlay(st.gt_xy[rows], {k: v["_xy"] for k, v in res.items()}, out / "fused_trajectories.png",
                      f"{name}: duty {duty:.0%}")
    _markdown(report, out)
    return report


def _markdown(r, out):
    s = r["support_step_err_in_windows"]
    L = [f"# Step 5 - {r['support']} in {r['schedule']} windows (baseline {r['baseline']})\n",
         f"{r['n_windows']} windows, duty cycle {r['duty_cycle']:.1%}, warm-up {r['warmup_s']} s, "
         f"{'warm' if r['warm_start'] else 'cold'} start, support compute {r['support_wall_s']:.0f} s.\n",
         f"Median step error inside windows: windowed {s['windowed_median']}, continuous {s['continuous_median']}, "
         f"baseline {s['baseline_median']} (m).\n",
         "| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |", "|---|---:|---:|---:|---:|"]
    for k, v in r["fused"].items():
        L.append(f"| {k} | {v['step_trans_err_m'].get('rmse', float('nan')):.4f} | {v['integrated_ate_m'].get('rmse', float('nan')):.1f} | "
                 f"{v['integrated_final_m']:.1f} | {v['drift_pct']['200m'].get('mean', float('nan')):.2f} |")
    (out / "windows_summary.md").write_text("\n".join(L) + "\n")
