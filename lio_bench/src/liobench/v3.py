"""v3 evaluation: 5-fold leave-one-block-out over the drive, paired comparisons.

The drive is cut into K contiguous time blocks (~2 km each). For each held-out block, anything
calibrated (yaw-rate bias, choice of heading source) is fitted on the other K-1 blocks only.
The held-out block is scored with the canonical protocol (audit.canonical: 200 m start fit,
then prediction). Pipelines are compared block by block (paired), which removes how hard each
block is from the comparison.
"""
from __future__ import annotations

import json

import numpy as np

from . import audit
from .correct import heading as H
from .correct.common import PoseTrack, baseline_track
from .correct.planar import body_up, planarize
from .dataset import _gt_heading
from .io import load_config, load_ground_truth, load_run, results_dir
from .bag import fit_sensor_clock, sensor_to_unix

K = 5
TAU = 5.0          # gravity filter time constant, chosen on the validation block (flat for 0.5-10 s)

SOURCES = {"faster_lio": ("faster_lio", "full"), "fast_lio": ("fast_lio", "full"), "dlio": ("dlio", "full"),
           "genz_lio": ("genz_lio", "full_ros"), "kiss_icp": ("kiss_icp", "full")}


def blocks(cfg: dict, k: int = K) -> list[tuple[float, float]]:
    gt = load_ground_truth(cfg)
    e = np.linspace(gt.t[0], gt.t[-1], k + 1)
    return [(float(e[i]), float(e[i + 1])) for i in range(k)]


class Source:
    """One estimator: raw poses, gravity-filtered poses, tilt-free steps, clock offset."""

    def __init__(self, name: str, algo: str, tag: str, cfg: dict, imu: np.ndarray, up: np.ndarray):
        self.name, self.algo, self.tag = name, algo, tag
        tk = baseline_track(algo, tag)
        self.t = tk.t
        self.raw = tk
        locked = "+vgicp_grav" in algo or "+grav_only" in algo      # already gravity-locked online
        self.G = PoseTrack(tk.t, tk.T if locked else H.gravity_filter(tk, imu, TAU))
        self.u = H.planar_steps(self.G, up)
        self.dt = audit.train_offset(load_run(algo, tag, cfg), cfg)
        self.tu = sensor_to_unix(self.t, fit_sensor_clock(cfg)) + self.dt
        self.x0 = self.G.T[0, :2, 3]
        self.yaw0 = float(np.arctan2(self.G.T[0, 1, 0], self.G.T[0, 0, 0]))

    def block_bias(self, cfg: dict, lo: float, hi: float) -> float:
        """Yaw-rate bias (rad/s) of this source inside one block: slope of (own yaw - GT course)."""
        gt = load_ground_truth(cfg)
        g = _gt_heading(gt)
        sp = np.r_[0, np.linalg.norm(np.diff(gt.p[:, :2], axis=0), axis=1) / np.diff(gt.t)]
        sel = (gt.t >= lo) & (gt.t <= hi) & (sp > 3.0) & (gt.t >= self.tu[0]) & (gt.t <= self.tu[-1])
        if sel.sum() < 50:
            return float("nan")
        e = np.interp(gt.t[sel], self.tu, np.cumsum(self.u[:, 2])) - g[sel]
        return float(np.polyfit(gt.t[sel] - gt.t[sel][0], e, 1)[0])


def load_sources(cfg: dict, extra: dict | None = None) -> dict[str, Source]:
    imu = H.imu_cache(cfg)["imu"]
    up = body_up(cfg)
    src = dict(SOURCES, **(extra or {}))
    return {k: Source(k, a, t, cfg, imu, up) for k, (a, t) in src.items()
            if (load_run(a, t, cfg) is not None)}


def pipelines(base: Source, S: dict[str, Source], cfg: dict, train_blocks: list[tuple[float, float]]):
    """All pipelines for one baseline; calibration uses only `train_blocks`. Returns {name: (t_sensor, T, dt)}."""
    def bias(src: Source) -> float:
        b = [(src.block_bias(cfg, lo, hi), hi - lo) for lo, hi in train_blocks]
        b = [(v, w) for v, w in b if np.isfinite(v)]
        return float(sum(v * w for v, w in b) / sum(w for _, w in b)) if b else 0.0
    t, u = base.t, base.u
    out = {"raw": (t, base.raw.T, base.dt),
           "planar (offline)": (t, planarize(base.raw, body_up(cfg)), base.dt),
           "G: gravity filter (online)": (t, base.G.T, base.dt)}
    biases = {k: bias(s) for k, s in S.items()}
    out["G + own yaw bias (calibrated)"] = (t, H.integrate(t, u, base.x0, base.yaw0, yaw_bias=biases[base.name]), base.dt)
    # least-biased heading source, chosen on the training blocks only
    best = min(biases, key=lambda k: abs(biases[k]))
    s = S[best]
    dy = H.resample_dyaw(s.t, s.u[:, 2], t) if s is not base else u[:, 2]
    out[f"G + yaw from least-biased source"] = (t, H.integrate(t, u, base.x0, base.yaw0, dyaw_override=dy), base.dt)
    dyb = H.resample_dyaw(s.t, s.u[:, 2] - biases[best] * np.r_[0, np.diff(s.t)], t)
    out[f"G + least-biased source, calibrated"] = (t, H.integrate(t, u, base.x0, base.yaw0, dyaw_override=dyb), base.dt)
    return out, biases, best


def run(cfg: dict | None = None, baselines=("faster_lio", "fast_lio"), extra: dict | None = None) -> dict:
    cfg = cfg or load_config()
    S = load_sources(cfg, extra)
    B = blocks(cfg)
    rep = {"blocks": B, "tau_s": TAU, "per_baseline": {}}
    for bname in baselines:
        base = S[bname]
        per = {}
        chosen = []
        for j, (lo, hi) in enumerate(B):
            train = [b for i, b in enumerate(B) if i != j]
            P, biases, best = pipelines(base, S, cfg, train)
            chosen.append({"block": j, "least_biased_source": best,
                           "bias_deg_s": {k: float(np.degrees(v)) for k, v in biases.items()}})
            for name, (t, T, dt) in P.items():
                tr = H.to_traj(name, t, T, cfg)
                per.setdefault(name, []).append(audit.canonical(tr, cfg, (lo, hi), dt))
        rep["per_baseline"][bname] = {"per_block": per, "calibration": chosen,
                                      "paired": paired(per, ref="G: gravity filter (online)")}
    (results_dir("v3") / "loo_blocks.json").write_text(json.dumps(rep, indent=1, default=float))
    return rep


def paired(per: dict, ref: str, keys=("ate_m", "final_m", "drift200_pct", "heading_err_end_deg")) -> dict:
    """Mean over blocks, and paired mean difference vs `ref` with its SE and sign count."""
    out = {}
    for name, rows in per.items():
        o = {}
        for k in keys:
            v = np.array([r.get(k, np.nan) for r in rows], float)
            if k == "heading_err_end_deg":
                v = np.abs(v)
            r0 = np.array([r.get(k, np.nan) for r in per[ref]], float)
            if k == "heading_err_end_deg":
                r0 = np.abs(r0)
            d = v - r0
            ok = np.isfinite(d)
            o[k] = {"mean": float(np.nanmean(v)), "per_block": v.round(3).tolist(),
                    "diff_vs_ref": float(np.nanmean(d)),
                    "se": float(np.nanstd(d[ok], ddof=1) / np.sqrt(ok.sum())) if ok.sum() > 1 else float("nan"),
                    "blocks_better": int(np.sum(d[ok] < 0))}
        out[name] = o
    return out


# --------------------------------------------------------------------------- combined pipelines

BLEND_GRID = (1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 300.0)


def combo(trans: Source, yaw: Source, cfg: dict, train_blocks, *, calibrate_yaw: bool = False,
          can_forward: bool = False, blend=None, raw: bool = False):
    """raw: the estimator's own poses, untouched. blend: None (heading increments taken from `yaw` directly), a time constant in s, or
    'select' (complementary blend of trans' heading with yaw's, tau chosen on the training blocks)."""
    if raw:
        return trans.t, trans.raw.T, trans.dt
    if blend == "select":
        scores = {}
        for tau in BLEND_GRID:
            t_, T_, dt_ = combo(trans, yaw, cfg, train_blocks, calibrate_yaw=calibrate_yaw,
                                can_forward=can_forward, blend=tau)
            tr = H.to_traj("sel", t_, T_, cfg)
            v = [audit.canonical(tr, cfg, b, dt_).get("ate_m", np.nan) for b in train_blocks]
            scores[tau] = float(np.nanmean(v))
        tau = min(scores, key=scores.get)
        out = combo(trans, yaw, cfg, train_blocks, calibrate_yaw=calibrate_yaw, can_forward=can_forward, blend=tau)
        combo.last_tau = tau
        return out
    """Translation steps from `trans`, heading increments from `yaw` (both tilt-free, online);
    optional training-block yaw-bias removal and CAN forward distance (training-block wheel calibration)."""
    t, u = trans.t, trans.u.copy()
    dy_src = yaw.u[:, 2].copy()
    if calibrate_yaw:
        b = [(yaw.block_bias(cfg, lo, hi), hi - lo) for lo, hi in train_blocks]
        b = [(v, w) for v, w in b if np.isfinite(v)]
        bias = sum(v * w for v, w in b) / sum(w for _, w in b) if b else 0.0
        dy_src = dy_src - bias * np.r_[0, np.diff(yaw.t)]
    dy = dy_src if yaw is trans else H.resample_dyaw(yaw.t, dy_src, t)
    if blend is not None:
        dy = H.blend_heading(t, u[:, 2], dy, float(blend))
    if can_forward:
        from .correct.wheel import load_calibration, wheel_steps
        cal = load_calibration(cfg, "faster_lio")
        w = wheel_steps(cfg, cal, t)
        ok = np.isfinite(w)
        u[ok, 0] = w[ok]
    return t, H.integrate(t, u, trans.x0, trans.yaw0, dyaw_override=dy), trans.dt


def run_combos(cfg: dict | None = None, extra: dict | None = None, combos: dict | None = None,
               out_name: str = "combos") -> dict:
    """Leave-one-block-out evaluation of named (translation, yaw, options) combinations."""
    cfg = cfg or load_config()
    S = load_sources(cfg, extra)
    B = blocks(cfg)
    per = {}
    for j, (lo, hi) in enumerate(B):
        train = [b for i, b in enumerate(B) if i != j]
        for name, (ts, ys, opts) in combos.items():
            if ts not in S or ys not in S:
                continue
            t, T, dt = combo(S[ts], S[ys], cfg, train, **opts)
            r = audit.canonical(H.to_traj(name, t, T, cfg), cfg, (lo, hi), dt)
            if opts.get("blend") == "select":
                r["tau_selected_s"] = combo.last_tau
            per.setdefault(name, []).append(r)
    ref = next(iter(combos))
    rep = {"blocks": B, "combos": {k: [v[0], v[1], v[2]] for k, v in combos.items()}, "per_block": per,
           "paired": paired(per, ref=ref)}
    (results_dir("v3") / f"{out_name}.json").write_text(json.dumps(rep, indent=1, default=float))
    return rep


# --------------------------------------------------------------------------- learned fusion vs the physics rule

def learned_vs_rule(cfg: dict | None = None, baseline: str = "faster_lio",
                    supports=("kiss_icp", "dlio", "genz_lio"), epochs: int = 300) -> dict:
    """Per held-out block: (a) the rule 'translation from the baseline, yaw from KISS-ICP',
    (b) static inverse-variance weights, (c) linear gating, (d) MLP-16 — (b-d) trained on the
    other blocks only (last 20 % of them for early stopping). All built from the same tilt-invariant
    step table and scored canonically."""
    import pandas as pd
    from . import fusion
    from .dataset import build
    from .geometry import integrate_increments
    from .io import Trajectory
    from scipy.spatial.transform import Rotation
    cfg = cfg or load_config()
    methods = [baseline, *supports]
    st = build(cfg, methods=methods)
    U = np.stack([st.inc[m] for m in methods], 1)
    mask = np.isfinite(U).all(2)
    y = st.gt
    ctx = fusion.CONTEXT_PREFIXES + (("genz_",) if "genz_lio" in supports else ())
    X, _ = fusion.make_features(st, methods, ctx)
    B = blocks(cfg)
    gap = int(round(cfg["split"]["gap_s"] * 10))
    per = {}

    def score(name, u, rows, lo, hi):
        u = pd.DataFrame(u[rows]).ffill().fillna(0).to_numpy()
        xy, yw = integrate_increments(u, st.gt_xy[rows[0]], st.gt_yaw[rows[0]])
        tr = Trajectory(name, st.t[rows], np.c_[xy, np.zeros(len(xy))],
                        Rotation.from_euler("z", yw[:, None]).as_quat(), {"world_frame": "gravity"})
        per.setdefault(name, []).append(audit.canonical(tr, cfg, (lo, hi), 0.0))

    for j, (lo, hi) in enumerate(B):
        te = np.flatnonzero((st.t >= lo) & (st.t <= hi))
        keep = np.ones(len(st.t), bool)
        keep[max(0, te[0] - gap):min(len(st.t), te[-1] + gap + 1)] = False
        tr_all = np.flatnonzero(keep)
        n_va = max(1, int(0.2 * len(tr_all)))
        tr, va = tr_all[:-n_va], tr_all[-n_va:]
        ib, ik = methods.index(baseline), methods.index("kiss_icp")
        rule = np.c_[np.nan_to_num(U[:, ib, :2]), np.nan_to_num(U[:, ik, 2])]
        score("rule: baseline translation + KISS yaw", rule, te, lo, hi)
        score("baseline alone (tilt-invariant)", np.nan_to_num(U[:, ib]), te, lo, hi)
        score("static inverse-variance", fusion.apply_static(U, mask, fusion.static_inverse_variance(U, y, mask, tr)), te, lo, hi)
        for label, hidden in (("linear gating", 0), ("MLP-16", 16)):
            fit = fusion.train_gating(X, U, mask, y, tr, va, epochs=epochs, seed=0, hidden=hidden)
            wt, wy = fusion.predict_weights(fit, X, mask)
            u = np.c_[(wt[:, :, None] * np.nan_to_num(U[:, :, :2])).sum(1), (wy * np.nan_to_num(U[:, :, 2])).sum(1)]
            score(label, u, te, lo, hi)
            score(f"{label} translation + KISS yaw", np.c_[u[:, :2], rule[:, 2]], te, lo, hi)
    rep = {"methods": methods, "blocks": B, "per_block": per,
           "paired": paired(per, ref="rule: baseline translation + KISS yaw")}
    (results_dir("v3") / f"learned_vs_rule_{baseline}.json").write_text(json.dumps(rep, indent=1, default=float))
    return rep
