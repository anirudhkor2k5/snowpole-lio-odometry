"""Step 4: a small model that decides, at every step, how much to trust each method.

Output per step: two weight vectors over the available methods (translation and
yaw), each a masked softmax, so a method that produced nothing at that step gets
weight 0. Fused increment = weighted mix of the methods' body-frame increments.
Trained against ground-truth increments (per-step Huber + a 2 s window-sum term
that penalises bias, which is what accumulates into drift).

Also here: the *trigger* model for step 5. It sees only what is available while
the support methods are OFF (baseline increments, sensor context, CAN) and
predicts the baseline's step error, i.e. when it is worth switching support on.

Everything is evaluated on chronological held-out blocks against: each method
alone, a uniform average, static inverse-variance weights (fit on train), and a
per-step oracle (best single method per step; uses ground truth, upper bound).
"""
from __future__ import annotations

import json
from dataclasses import dataclass

import numpy as np
import pandas as pd
import torch
from torch import nn

from . import plots
from .dataset import StepTable, build
from .geometry import integrate_increments
from .io import load_config, results_dir
from .metrics import segment_drift, stats

YAW_LEVER_M = 10.0          # yaw error weighted like the lateral error it causes 10 m ahead
CONTEXT_PREFIXES = ("scan_", "can_")


# --------------------------------------------------------------------------- splits

def chronological_split(n: int, t: np.ndarray, train: float, val: float, gap_s: float):
    i_tr = int(n * train)
    i_va = int(n * (train + val))
    gap = int(round(gap_s * 10))
    tr = np.arange(0, i_tr)
    va = np.arange(min(i_tr + gap, n), i_va)
    te = np.arange(min(i_va + gap, n), n)
    return tr, va, te


def blocked_folds(n: int, k: int, gap_s: float):
    """k contiguous test blocks; train on the rest minus a gap on each side."""
    edges = np.linspace(0, n, k + 1).astype(int)
    gap = int(round(gap_s * 10))
    for i in range(k):
        te = np.arange(edges[i], edges[i + 1])
        keep = np.ones(n, bool)
        keep[max(0, edges[i] - gap):min(n, edges[i + 1] + gap)] = False
        tr_all = np.flatnonzero(keep)
        # last 20 % of the training rows (chronologically) for early stopping
        n_va = max(1, int(0.2 * len(tr_all)))
        yield tr_all[:-n_va], tr_all[-n_va:], te


# --------------------------------------------------------------------------- features

def _rolling(x: np.ndarray, w: int) -> np.ndarray:
    """Causal rolling mean over the last w rows (NaN-aware)."""
    s = pd.DataFrame(x).rolling(w, min_periods=1).mean().to_numpy()
    return s


def make_features(st: StepTable, methods: list[str], context_prefixes=CONTEXT_PREFIXES,
                  include_disagreement: bool = True, inc: dict | None = None,
                  ctx_off: dict | None = None) -> tuple[np.ndarray, list[str]]:
    """Causal features. `inc` overrides st.inc (e.g. a support only available in
    windows); `ctx_off` maps a context prefix to a boolean row mask where that
    context is unavailable (e.g. GenZ internals while GenZ is not running)."""
    inc = inc or st.inc
    cols, names = [], []
    for c in st.ctx.columns:
        if c.startswith(context_prefixes):
            v = st.ctx[c].to_numpy(float).copy()
            for pre, off in (ctx_off or {}).items():
                if c.startswith(pre):
                    v[off] = np.nan
            cols.append(v); names.append(c)
    U = np.stack([inc[m] for m in methods])              # (M, N, 3)
    med = np.nanmedian(U, axis=0) if len(methods) > 1 else None
    for i, m in enumerate(methods):
        u = U[i]
        avail = np.isfinite(u).all(1).astype(float)
        cols += [avail]; names += [f"{m}_available"]
        uu = np.nan_to_num(u)
        for j, k in enumerate(("fwd", "lat", "dyaw")):
            cols.append(uu[:, j]); names.append(f"{m}_{k}")
            cols.append(_rolling(u[:, j:j + 1], 10)[:, 0]); names.append(f"{m}_{k}_1s")
        cols.append(_rolling(np.abs(np.diff(np.r_[uu[:1, 0], uu[:, 0]]))[:, None], 10)[:, 0]); names.append(f"{m}_fwd_jerk_1s")
        if include_disagreement and med is not None:
            d = u - med
            for j, k in enumerate(("fwd", "lat", "dyaw")):
                cols.append(np.nan_to_num(d[:, j])); names.append(f"{m}_dev_{k}")
                cols.append(np.nan_to_num(_rolling(np.abs(d[:, j:j + 1]), 20)[:, 0])); names.append(f"{m}_absdev_{k}_2s")
    X = np.stack(cols, 1).astype(np.float32)
    return X, names


class Standardizer:
    def fit(self, X):
        self.mu = np.nanmean(X, 0)
        self.sd = np.nanstd(X, 0) + 1e-6
        self.mu[~np.isfinite(self.mu)] = 0
        return self

    def __call__(self, X):
        Z = (X - self.mu) / self.sd
        return np.nan_to_num(np.clip(Z, -8, 8)).astype(np.float32)


# --------------------------------------------------------------------------- model

class GatingMLP(nn.Module):
    """hidden = 0: linear gating (softmax of a linear function of the features)."""

    def __init__(self, n_in: int, n_methods: int, hidden: int = 64, dropout: float = 0.1):
        super().__init__()
        if hidden > 0:
            self.body = nn.Sequential(nn.Linear(n_in, hidden), nn.GELU(), nn.Dropout(dropout),
                                      nn.Linear(hidden, hidden), nn.GELU(), nn.Dropout(dropout))
        else:
            self.body = nn.Dropout(dropout)
        self.head_t = nn.Linear(hidden or n_in, n_methods)
        self.head_y = nn.Linear(hidden or n_in, n_methods)

    def forward(self, x, mask):
        h = self.body(x)
        neg = torch.finfo(h.dtype).min
        wt = torch.softmax(self.head_t(h).masked_fill(~mask, neg), -1)
        wy = torch.softmax(self.head_y(h).masked_fill(~mask, neg), -1)
        return wt, wy


def fuse(U: torch.Tensor, wt: torch.Tensor, wy: torch.Tensor) -> torch.Tensor:
    """U (N, M, 3) with NaN->0; weights (N, M)."""
    tr = (wt.unsqueeze(-1) * U[:, :, :2]).sum(1)
    yw = (wy * U[:, :, 2]).sum(1, keepdim=True)
    return torch.cat([tr, yw], 1)


def _loss(pred, target, window: int = 20):
    scale = torch.tensor([1.0, 1.0, YAW_LEVER_M], device=pred.device)
    e = (pred - target) * scale
    step = nn.functional.huber_loss(e, torch.zeros_like(e), delta=0.5)
    n = (len(e) // window) * window
    if n == 0:
        return step
    w = e[:n].reshape(-1, window, 3).sum(1)
    return step + 0.1 * nn.functional.huber_loss(w, torch.zeros_like(w), delta=2.0)


@dataclass
class FitResult:
    model: GatingMLP
    scaler: Standardizer
    feature_names: list[str]
    history: list[dict]


def train_gating(X, U, mask, y, tr, va, epochs=400, lr=2e-3, wd=1e-3, hidden=64, seed=0, patience=40,
                 chunk=200, verbose=False, variants=None, variant_p=None) -> FitResult:
    """`variants`: optional list of (X_v, mask_v) with some support methods switched
    off; each training chunk draws one (availability dropout), so the model also
    learns what to do when a support method is not running. Validation uses the
    full-availability X."""
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    sc = Standardizer().fit(X[tr])
    Xs = torch.tensor(sc(X))
    Ut = torch.tensor(np.nan_to_num(U), dtype=torch.float32)
    Mt = torch.tensor(mask)
    yt = torch.tensor(y, dtype=torch.float32)
    var_t = [(torch.tensor(sc(Xv)), torch.tensor(mv)) for Xv, mv in (variants or [])]
    model = GatingMLP(X.shape[1], U.shape[1], hidden)
    opt = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=wd)
    best, best_state, bad, hist = np.inf, None, 0, []
    tr_rows = tr[np.isfinite(y[tr]).all(1) & mask[tr].any(1)]
    va_rows = va[np.isfinite(y[va]).all(1) & mask[va].any(1)]
    starts = np.arange(0, len(tr_rows), chunk)
    for ep in range(epochs):
        model.train()
        rng.shuffle(starts)
        for s in starts:                       # contiguous chunks keep the window-sum term meaningful
            idx = torch.tensor(tr_rows[s:s + chunk])
            Xc, Mc = Xs, Mt
            if var_t:
                k = rng.choice(len(var_t) + 1, p=variant_p)
                if k > 0:
                    Xc, Mc = var_t[k - 1]
            m_idx = Mc[idx]
            keep = m_idx.any(1)
            if not bool(keep.any()):
                continue
            idx = idx[keep]
            wt, wy = model(Xc[idx], Mc[idx])
            loss = _loss(fuse(Ut[idx], wt, wy), yt[idx])
            opt.zero_grad(); loss.backward(); opt.step()
        model.eval()
        with torch.no_grad():
            idx = torch.tensor(va_rows)
            wt, wy = model(Xs[idx], Mt[idx])
            vl = float(_loss(fuse(Ut[idx], wt, wy), yt[idx]))
        hist.append({"epoch": ep, "val_loss": vl})
        if vl < best - 1e-5:
            best, bad = vl, 0
            best_state = {k: v.clone() for k, v in model.state_dict().items()}
        else:
            bad += 1
            if bad > patience:
                break
        if verbose and ep % 50 == 0:
            print(f"epoch {ep} val {vl:.4f}")
    model.load_state_dict(best_state)
    return FitResult(model, sc, [], hist)


def predict_weights(fit: FitResult, X, mask):
    fit.model.eval()
    with torch.no_grad():
        wt, wy = fit.model(torch.tensor(fit.scaler(X)), torch.tensor(mask))
    return wt.numpy(), wy.numpy()


def support_off(st: StepTable, methods, supports_off, ctx_prefixes, off_rows=None):
    """Features and mask with the given supports unavailable on `off_rows` (default: all rows)."""
    n = len(st.t)
    off_rows = np.ones(n, bool) if off_rows is None else off_rows
    inc = dict(st.inc)
    ctx_off = {}
    for m in supports_off:
        u = inc[m].copy()
        u[off_rows] = np.nan
        inc[m] = u
        if m == "genz_lio":
            ctx_off["genz_"] = off_rows
    X, _ = make_features(st, methods, ctx_prefixes, inc=inc, ctx_off=ctx_off)
    U = np.stack([inc[m] for m in methods], 1)
    return X, np.isfinite(U).all(2), U


def dropout_variants(st, methods, supports, ctx_prefixes, mask):
    """Every non-empty subset of supports switched off; full availability keeps p = 0.5."""
    from itertools import combinations
    subsets = [c for r in range(1, len(supports) + 1) for c in combinations(supports, r)]
    variants = []
    for sub in subsets:
        Xv, mv, _ = support_off(st, methods, sub, ctx_prefixes)
        variants.append((Xv, mv & mask))
    p = [0.5] + [0.5 / len(subsets)] * len(subsets) if subsets else None
    return variants, p


# --------------------------------------------------------------------------- comparators

def static_inverse_variance(U, y, mask, tr):
    """Per-component weights ∝ 1 / train error variance of each method."""
    M = U.shape[1]
    w = np.zeros((M, 3))
    for m in range(M):
        ok = tr[mask[tr, m] & np.isfinite(y[tr]).all(1)]
        e = U[ok, m] - y[ok]
        w[m] = 1.0 / (np.var(e, 0) + 1e-9)
    return w / w.sum(0, keepdims=True)


def apply_static(U, mask, w):
    W = mask[:, :, None] * w[None]
    W = W / np.clip(W.sum(1, keepdims=True), 1e-12, None)
    return (np.nan_to_num(U) * W).sum(1)


def oracle(U, y, mask):
    e = np.linalg.norm(np.nan_to_num(U[:, :, :2] - y[:, None, :2]), axis=2) + np.abs(np.nan_to_num(U[:, :, 2] - y[:, None, 2])) * YAW_LEVER_M
    e[~mask] = np.inf
    best = np.argmin(e, 1)
    return np.nan_to_num(U)[np.arange(len(U)), best], best


# --------------------------------------------------------------------------- evaluation

def evaluate_block(name: str, u_hat: np.ndarray, st: StepTable, rows: np.ndarray) -> dict:
    """Step errors and the trajectory obtained by integrating u_hat over `rows`.

    The integration starts from the ground-truth pose at the block start (as a
    deployment would start from a GNSS fix) and is never corrected afterwards.
    """
    y = st.gt[rows]
    ok = np.isfinite(y).all(1) & np.isfinite(u_hat[rows]).all(1)
    e_t = np.linalg.norm(u_hat[rows][ok, :2] - y[ok, :2], axis=1)
    e_y = np.degrees(np.abs(u_hat[rows][ok, 2] - y[ok, 2]))
    xy, _ = integrate_increments(u_hat[rows], st.gt_xy[rows[0]], st.gt_yaw[rows[0]])
    ref = st.gt_xy[rows]
    err = np.linalg.norm(xy - ref, axis=1)
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(ref, axis=0), axis=1))]
    drift = {f"{L}m": stats(segment_drift(xy, ref, d, L)[1]) for L in (100, 200, 400)}
    return {"name": name, "step_trans_err_m": stats(e_t), "step_yaw_err_deg": stats(e_y),
            "integrated_ate_m": stats(err), "integrated_final_m": float(err[-1]), "length_m": float(d[-1]),
            "drift_pct": drift, "_xy": xy}


def _strip(d):
    return {k: v for k, v in d.items() if not k.startswith("_")}


# Model ladder, simplest first. Chosen by blocked CV with the one-standard-error rule:
# the simplest model whose CV score is within one standard error of the best one.
# "best_single" = the one method with the lowest training step error (no mixing at all).
LADDER = [("best_single", "single"), ("static_inv_var", None), ("linear_gating", 0), ("mlp16", 16), ("mlp64", 64)]
CV_SCORE = "drift_200m_pct"      # lower is better


def best_single_weights(U, y, mask, tr):
    """One-hot weights on the method with the lowest training step error (translation + yaw lever)."""
    M = U.shape[1]
    err = np.full(M, np.inf)
    for m in range(M):
        ok = tr[mask[tr, m] & np.isfinite(y[tr]).all(1)]
        if len(ok):
            e = np.linalg.norm(U[ok, m, :2] - y[ok, :2], axis=1) + YAW_LEVER_M * np.abs(U[ok, m, 2] - y[ok, 2])
            err[m] = np.mean(e)
    w = np.full((M, 3), 1e-6)           # tiny weight elsewhere: falls back to the others only when it has no output
    w[int(np.argmin(err))] = 1.0
    return w / w.sum(0, keepdims=True)


def paired_one_se(per_fold: dict[str, np.ndarray], order: list[str]) -> tuple[str, dict]:
    """Simplest model whose per-fold disadvantage against the best model is within one SE of
    the paired differences. Pairing by fold removes how hard each fold is, which otherwise
    dominates the spread on a single drive with one very hard stretch."""
    best = min(order, key=lambda n: np.nanmean(per_fold[n]))
    info = {}
    for n in order:
        d = per_fold[n] - per_fold[best]
        k = np.isfinite(d).sum()
        se = float(np.nanstd(d, ddof=1) / np.sqrt(k)) if k > 1 else np.inf
        info[n] = {"mean_diff_vs_best": float(np.nanmean(d)), "se": se,
                   "folds_better_than_best": int(np.nansum(d < 0))}
    chosen = next(n for n in order if info[n]["mean_diff_vs_best"] <= info[n]["se"] or n == best)
    return chosen, {"best_by_mean": best, "paired": info}


def _context_columns(names, ctx_prefixes):
    return np.array([n.startswith(ctx_prefixes) for n in names])


def run(cfg: dict | None = None, baseline: str = "fast_lio", supports: list[str] | None = None,
        cv: bool = True, epochs: int = 400, seed: int = 0, n_seeds: int = 3, out_name: str | None = None) -> dict:
    cfg = cfg or load_config()
    out = results_dir("fusion", out_name or baseline)
    supports = supports if supports is not None else ["dlio", "genz_lio"]
    st = build(cfg, methods=[baseline] + list(supports))
    supports = [s_ for s_ in supports if s_ in st.inc]
    if baseline not in st.inc:
        raise RuntimeError(f"no full run of {baseline}")
    methods = [baseline] + supports
    U = np.stack([st.inc[m] for m in methods], 1)          # (N, M, 3)
    mask = np.isfinite(U).all(2)
    y = st.gt
    ctx_prefixes = CONTEXT_PREFIXES + (("genz_",) if "genz_lio" in supports else ())
    X, names = make_features(st, methods, ctx_prefixes)
    variants, variant_p = dropout_variants(st, methods, supports, ctx_prefixes, mask)
    sp = cfg["split"]

    def fused_from(fit_or_w, X_=X):
        if isinstance(fit_or_w, np.ndarray):
            return apply_static(U, mask, fit_or_w), None, None
        wt, wy = predict_weights(fit_or_w, X_, mask)
        return np.c_[(wt[:, :, None] * np.nan_to_num(U[:, :, :2])).sum(1), (wy * np.nan_to_num(U[:, :, 2])).sum(1)], wt, wy

    def fit_model(hidden, tr, va, seed_, X_=X, variants_=variants):
        if hidden == "single":
            return best_single_weights(U, y, mask, tr)
        if hidden is None:
            return static_inverse_variance(U, y, mask, tr)
        return train_gating(X_, U, mask, y, tr, va, epochs=epochs, seed=seed_, hidden=hidden,
                            variants=variants_, variant_p=variant_p)

    def comparators(tr, te):
        res = {}
        for i, m in enumerate(methods):
            um = pd.DataFrame(U[:, i].copy()).ffill().fillna(0).to_numpy()   # hold the last good increment
            res[m] = evaluate_block(m, um, st, te)
        res["uniform_mean"] = evaluate_block("uniform_mean", apply_static(U, mask, np.ones((len(methods), 3)) / len(methods)), st, te)
        u_oracle, best = oracle(U, y, mask)
        res["oracle_best_per_step"] = evaluate_block("oracle_best_per_step", u_oracle, st, te)
        return res, best

    report = {"baseline": baseline, "methods": methods, "n_features": len(names), "features": names,
              "ladder": [n for n, _ in LADDER], "cv_score": CV_SCORE}

    # ---- 1. model selection by blocked CV (one-standard-error rule)
    chosen = "static_inv_var"
    if cv:
        folds = []
        for k, (ftr, fva, fte) in enumerate(blocked_folds(len(y), sp["folds"], sp["gap_s"])):
            fres, _ = comparators(ftr, fte)
            for name, hidden in LADDER:
                u_hat, _, _ = fused_from(fit_model(hidden, ftr, fva, seed))
                fres[name] = evaluate_block(name, u_hat, st, fte)
            folds.append({n: _strip(r) for n, r in fres.items()})
        agg = {}
        for n in folds[0]:
            d200 = np.array([f[n]["drift_pct"]["200m"].get("mean", np.nan) for f in folds])
            agg[n] = {"step_trans_err_rmse_mean": float(np.mean([f[n]["step_trans_err_m"]["rmse"] for f in folds])),
                      "integrated_ate_rmse_mean": float(np.mean([f[n]["integrated_ate_m"]["rmse"] for f in folds])),
                      "drift_200m_pct": float(np.nanmean(d200)),
                      "drift_200m_pct_se": float(np.nanstd(d200, ddof=1) / np.sqrt(np.isfinite(d200).sum()))}
        order = [n for n, _ in LADDER]
        per_fold = {n: np.array([f[n]["drift_pct"]["200m"].get("mean", np.nan) for f in folds]) for n in folds[0]}
        chosen, sel = paired_one_se(per_fold, order)
        report["cv"] = {"folds": len(folds), "aggregate": agg, "per_fold": folds, "chosen": chosen,
                        "per_fold_drift_200m_pct": {n: v.round(2).tolist() for n, v in per_fold.items()}, **sel}
    hidden_chosen = dict(LADDER)[chosen]
    learned = hidden_chosen not in (None, "single")

    # ---- 2. held-out block: chosen model trained on train, early-stopped on val, scored once on test
    tr, va, te = chronological_split(len(y), st.t, sp["train"], sp["val"], sp["gap_s"])
    res, best = comparators(tr, te)
    w_static = static_inverse_variance(U, y, mask, tr)
    res["static_inv_var"] = evaluate_block("static_inv_var", apply_static(U, mask, w_static), st, te)
    w_single = best_single_weights(U, y, mask, tr)
    res["best_single"] = evaluate_block("best_single", apply_static(U, mask, w_single), st, te)
    seeds = [seed + i for i in range(n_seeds if learned else 1)]
    fits = [fit_model(hidden_chosen, tr, va, sd) for sd in seeds]
    fit = fits[0]
    u_chosen, wt, wy = fused_from(fit)
    res[f"chosen:{chosen}"] = evaluate_block(chosen, u_chosen, st, te)
    seed_scores = [evaluate_block(chosen, fused_from(f)[0], st, te)["drift_pct"]["200m"].get("mean") for f in fits]
    if not learned:
        u_chosen, wt, wy = fused_from(fit)

    # ---- 3. overfitting checks
    checks = {"seed_spread_test_drift_200m_pct": seed_scores}
    # train / val / test gap of the chosen model (step error, same metric everywhere)
    checks["gap_step_err_rmse_m"] = {b: evaluate_block(chosen, u_chosen, st, r)["step_trans_err_m"].get("rmse")
                                     for b, r in (("train", tr), ("val", va), ("test", te))}
    # shuffled-context control: context columns rolled by half the drive (marginals kept,
    # alignment with the errors destroyed). If the real model is not better on test, the
    # context dependence it learned is not real.
    if learned:
        cc = _context_columns(names, ctx_prefixes)
        Xc = X.copy()
        Xc[:, cc] = np.roll(X[:, cc], len(X) // 2, axis=0)
        var_c = [(np.where(cc[None], np.roll(Xv, len(X) // 2, axis=0), Xv), mv) for Xv, mv in variants]
        fit_c = fit_model(hidden_chosen, tr, va, seed, X_=Xc, variants_=var_c)
        res["control:shuffled_context"] = evaluate_block("shuffled_context", fused_from(fit_c, Xc)[0], st, te)
    # route overlap: is the test road also in the training block (e.g. an out-and-back drive)?
    from scipy.spatial import cKDTree
    dmin = cKDTree(st.gt_xy[tr]).query(st.gt_xy[te])[0]
    checks["test_within_25m_of_train_route_frac"] = float((dmin < 25).mean())

    def d200(n):
        return res[n]["drift_pct"]["200m"].get("mean", np.nan)
    verdict = []
    if not learned:
        verdict.append(f"CV picked `{chosen}`: no learned model beat it by more than one paired SE.")
    else:
        for ref in ("best_single", "static_inv_var"):
            if d200(f"chosen:{chosen}") >= d200(ref):
                verdict.append(f"On test the learned model does not beat `{ref}`.")
        if "control:shuffled_context" in res and d200(f"chosen:{chosen}") >= d200("control:shuffled_context"):
            verdict.append("On test the learned model does not beat its shuffled-context control: "
                           "the context dependence is not supported by held-out data.")
        if not verdict:
            verdict.append("Learned model beats best-single, static weights and its shuffled-context control on test.")
    if checks["test_within_25m_of_train_route_frac"] > 0.2:
        verdict.append("Part of the test road was also driven in training: test numbers may be optimistic.")
    checks["verdict"] = verdict

    w_fixed = w_single if hidden_chosen == "single" else w_static
    weights = pd.DataFrame(index=np.arange(len(st.t)))
    if wt is not None:
        weights = pd.DataFrame(wt, columns=[f"w_trans_{m}" for m in methods]).join(
            pd.DataFrame(wy, columns=[f"w_yaw_{m}" for m in methods]))
    else:
        for i, m in enumerate(methods):
            weights[f"w_trans_{m}"] = np.where(mask[:, i], w_fixed[i, 0], 0.0)
            weights[f"w_yaw_{m}"] = np.where(mask[:, i], w_fixed[i, 2], 0.0)
    weights.insert(0, "t", st.t)
    weights["split"] = "train"
    weights.loc[va, "split"] = "val"
    weights.loc[te, "split"] = "test"
    weights.to_csv(out / "weights_holdout.csv", index=False)
    ck = {"kind": "gating" if learned else "static", "chosen": chosen, "hidden": hidden_chosen if learned else None,
          "features": names, "methods": methods, "static_weights": w_fixed}
    if learned:
        ck.update(state_dict=fit.model.state_dict(), mu=fit.scaler.mu, sd=fit.scaler.sd)
    torch.save(ck, out / "gating_model.pt")

    report["holdout"] = {"train": [int(tr[0]), int(tr[-1])], "val": [int(va[0]), int(va[-1])], "test": [int(te[0]), int(te[-1])],
                         "chosen": chosen, "results": {k: _strip(v) for k, v in res.items()},
                         "static_weights": {m: w_static[i].round(3).tolist() for i, m in enumerate(methods)},
                         "oracle_share": {m: float((best[te] == i).mean()) for i, m in enumerate(methods)},
                         "mean_test_weights": weights[weights.split == "test"].drop(columns=["t", "split"]).mean().round(3).to_dict()}
    report["overfitting_checks"] = checks

    trig = train_trigger(st, baseline, tr, va, te)
    report["trigger"] = trig["report"]
    pd.DataFrame({"t": st.t, "pred_baseline_step_err": trig["pred"], "true_baseline_step_err": trig["true"]}).to_csv(
        out / "trigger_predictions.csv", index=False)

    (out / "fusion_summary.json").write_text(json.dumps(report, indent=2, default=float))
    _plots(st, res, weights, methods, te, out)
    _markdown(report, out)
    return report


# --------------------------------------------------------------------------- trigger model (for step 5)

def baseline_step_error(st: StepTable, baseline: str) -> np.ndarray:
    u, y = st.inc[baseline], st.gt
    return np.linalg.norm(u[:, :2] - y[:, :2], axis=1) + YAW_LEVER_M * np.abs(u[:, 2] - y[:, 2])


def trigger_features(st: StepTable, baseline: str):
    """Only what exists while support methods are off: sensor context, CAN, the baseline itself."""
    return make_features(st, [baseline], CONTEXT_PREFIXES, include_disagreement=False)


def train_trigger(st: StepTable, baseline: str, tr, va, te) -> dict:
    from sklearn.ensemble import HistGradientBoostingRegressor
    from sklearn.metrics import roc_auc_score

    X, names = trigger_features(st, baseline)
    e = baseline_step_error(st, baseline)
    # Predict the baseline's RELATIVE error (per metre driven) over the NEXT second, from features
    # up to now. Absolute error grows with speed, so predicting it mostly predicts speed.
    step_len = np.linalg.norm(st.gt[:, :2], axis=1)
    rel = e / np.maximum(step_len, 0.2)
    target = pd.Series(rel).rolling(10, min_periods=5).mean().shift(-10).to_numpy()
    ok_tr = tr[np.isfinite(target[tr])]
    ok_va = va[np.isfinite(target[va])]
    # boosting rounds chosen on the chronological validation block (a random validation
    # fraction would leak neighbouring, near-identical frames into early stopping)
    best = None
    for n_iter in (25, 50, 100, 200, 400):
        r = HistGradientBoostingRegressor(max_depth=3, max_iter=n_iter, learning_rate=0.05, l2_regularization=1.0,
                                          min_samples_leaf=50, early_stopping=False, random_state=0)
        r.fit(X[ok_tr], np.log1p(target[ok_tr]))
        sc = pd.Series(r.predict(X[ok_va])).corr(pd.Series(np.log1p(target[ok_va])), method="spearman")
        if best is None or sc > best[0]:
            best = (sc, n_iter, r)
    reg = best[2]
    pred = np.expm1(reg.predict(X))
    rep = {"target": "next-1 s baseline step error per metre driven", "features": names, "n_iter_chosen_on_val": best[1]}
    # control: the same model on the baseline's own speed only
    sp_col = [names.index(f"{baseline}_fwd_1s")]
    ctrl = HistGradientBoostingRegressor(max_depth=3, max_iter=best[1], learning_rate=0.05, l2_regularization=1.0,
                                         min_samples_leaf=50, early_stopping=False, random_state=0)
    ctrl.fit(X[ok_tr][:, sp_col], np.log1p(target[ok_tr]))
    pred_speed = np.expm1(ctrl.predict(X[:, sp_col]))
    for nm, rows in (("val", va), ("test", te)):
        ok = rows[np.isfinite(target[rows])]
        if len(ok) < 10:
            continue
        thr = np.nanpercentile(target[tr], 75)     # "bad" = worse than the training 75th percentile
        lab = target[ok] > thr
        auc = lambda p: float(roc_auc_score(lab, p[ok])) if 0 < lab.mean() < 1 else None
        rep[nm] = {"spearman": float(pd.Series(pred[ok]).corr(pd.Series(target[ok]), method="spearman")),
                   "bad_threshold": float(thr), "bad_rate": float(lab.mean()), "roc_auc_bad": auc(pred),
                   "speed_only_control": {
                       "spearman": float(pd.Series(pred_speed[ok]).corr(pd.Series(target[ok]), method="spearman")),
                       "roc_auc_bad": auc(pred_speed)}}
    return {"model": reg, "pred": pred, "true": target, "report": rep}


# --------------------------------------------------------------------------- output

def _plots(st, res, weights, methods, te, out):
    ref = st.gt_xy[te]
    tracks = {k: v["_xy"] for k, v in res.items() if k in methods or k in ("learned_gating", "static_inv_var")}
    plots.map_overlay(ref, tracks, out / "test_block_trajectories.png", "test block, integrated from its first pose")
    tt = weights["t"].to_numpy() - weights["t"].iloc[0]
    ys = {m: weights[f"w_trans_{m}"].rolling(20, min_periods=1).mean() for m in methods}
    plots.series(tt, ys, out / "translation_weights_vs_time.png", "time [s]", "weight (2 s mean)",
                 "learned translation weights (train | val | test)")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 7))
    w = weights[f"w_trans_{methods[0]}"].to_numpy()
    sc = ax.scatter(st.gt_xy[:, 0], st.gt_xy[:, 1], c=w, s=3, cmap="coolwarm_r", vmin=0, vmax=1)
    fig.colorbar(sc, ax=ax, label=f"weight on {methods[0]} (baseline)")
    ax.set_aspect("equal"); ax.set_title("where the model trusts the baseline")
    plots.save(fig, out / "map_baseline_weight.png")


def _markdown(rep, out):
    h = rep["holdout"]
    L = [f"# Step 4 - learned fusion, baseline = {rep['baseline']}\n",
         f"Methods: {rep['methods']}. {rep['n_features']} causal features. Test rows {h['test']}.\n",
         "## Held-out (last block of the drive)\n",
         "| method | step err RMSE m | yaw err median deg | integrated ATE RMSE m | final m | drift 200 m % |",
         "|---|---:|---:|---:|---:|---:|"]
    for n, r in h["results"].items():
        L.append(f"| {n} | {r['step_trans_err_m'].get('rmse', np.nan):.4f} | {r['step_yaw_err_deg'].get('median', np.nan):.3f} | "
                 f"{r['integrated_ate_m'].get('rmse', np.nan):.1f} | {r['integrated_final_m']:.1f} | "
                 f"{r['drift_pct']['200m'].get('mean', np.nan):.2f} |")
    L.append(f"\n**Model chosen by CV (one-SE rule): `{h['chosen']}`.**")
    oc = rep.get("overfitting_checks", {})
    if oc:
        L.append("\n## Overfitting checks\n")
        for v in oc.get("verdict", []):
            L.append(f"- {v}")
        L.append(f"- Step error RMSE train / val / test: {oc.get('gap_step_err_rmse_m')}")
        L.append(f"- Test drift 200 m % across seeds: {oc.get('seed_spread_test_drift_200m_pct')}")
        L.append(f"- Fraction of the test road within 25 m of the training route: {oc.get('test_within_25m_of_train_route_frac'):.2f}")
    L.append(f"\nMean test weights: {h['mean_test_weights']}\n\nStatic inverse-variance weights (fwd, lat, yaw): {h['static_weights']}\n")
    L.append(f"Oracle (best single method per step) share on test: {h['oracle_share']}\n")
    if "cv" in rep:
        L.append(f"## Blocked {rep['cv']['folds']}-fold CV (mean over folds)\n")
        L.append("| method | step err RMSE m | integrated ATE RMSE m | drift 200 m % | ± SE |\n|---|---:|---:|---:|---:|")
        for n, a in rep["cv"]["aggregate"].items():
            L.append(f"| {n} | {a['step_trans_err_rmse_mean']:.4f} | {a['integrated_ate_rmse_mean']:.1f} | "
                     f"{a['drift_200m_pct']:.2f} | {a['drift_200m_pct_se']:.2f} |")
        L.append("\nPer-fold drift 200 m % (folds in time order):\n")
        for n, v in rep["cv"]["per_fold_drift_200m_pct"].items():
            L.append(f"- `{n}`: {v}")
        L.append(f"\nPaired against the best by mean (`{rep['cv']['best_by_mean']}`), simplest first:\n")
        L.append("| model | mean diff (pp) | paired SE | folds better than best |\n|---|---:|---:|---:|")
        for n, i in rep["cv"]["paired"].items():
            L.append(f"| {n} | {i['mean_diff_vs_best']:+.2f} | {i['se']:.2f} | {i['folds_better_than_best']} |")
        L.append(f"\n-> chosen `{rep['cv']['chosen']}` (simplest within one paired SE of the best).")
    L.append(f"\n## Trigger model (baseline-only features -> baseline error next 1 s)\n\n{json.dumps({k: v for k, v in rep['trigger'].items() if k != 'features'}, indent=1)}\n")
    (out / "fusion_summary.md").write_text("\n".join(L) + "\n")
