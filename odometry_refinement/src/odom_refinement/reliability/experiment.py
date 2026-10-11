"""Train and score reliability models for one synchronized trajectory."""

from __future__ import annotations

import numpy as np

from odom_refinement.evaluation.reliability_metrics import (
    average_precision,
    binary_counts,
    brier_score,
    expected_calibration_error,
    false_alarms_per_minute,
    negative_log_likelihood,
    roc_auc,
    threshold_for_false_alarms,
)
from odom_refinement.features.odometry import FEATURE_GROUPS, FEATURE_NAMES, assert_feature_names_are_causal
from odom_refinement.reliability.analytical import analytical_scores_with_train_scale
from odom_refinement.reliability.models import (
    apply_normalizer,
    chronological_split,
    fit_logistic,
    fit_mlp,
    fit_normalizer,
    fit_xgboost,
    predict_logistic,
    predict_mlp,
    predict_xgboost,
)


def _duration(time_s: np.ndarray, mask: np.ndarray) -> float:
    chosen = time_s[mask]
    if len(chosen) < 2:
        return 0.0
    return float(chosen[-1] - chosen[0])


def _classification_report(y, scores, mask, time_s, threshold: float) -> dict:
    counts = binary_counts(y, scores, threshold, mask)
    duration = _duration(time_s, mask)
    return {
        "threshold": threshold,
        **counts,
        "false_alarms_per_minute": false_alarms_per_minute(counts["fp"], duration),
        "pr_auc": average_precision(y, scores, mask),
        "roc_auc": roc_auc(y, scores, mask),
        "brier": brier_score(y, scores, mask),
        "nll": negative_log_likelihood(y, scores, mask),
        "calibration": expected_calibration_error(y, scores, mask),
        "duration_s": duration,
    }


def _rows(split: np.ndarray, feature_valid: np.ndarray, label_usable: np.ndarray, name: str) -> np.ndarray:
    return (split == name) & feature_valid & label_usable


def run_reliability_case(
    time_s: np.ndarray,
    features: np.ndarray,
    feature_valid: np.ndarray,
    labels: np.ndarray,
    label_usable: np.ndarray,
    quality_weight: np.ndarray,
    *,
    gap_s: float = 20.0,
    seed: int = 0,
    false_alarm_target_per_minute: float = 1.0,
) -> dict:
    assert_feature_names_are_causal(FEATURE_NAMES)
    split = chronological_split(time_s, gap_s=gap_s)
    train = _rows(split, feature_valid, label_usable, "train")
    val = _rows(split, feature_valid, label_usable, "val")
    test = _rows(split, feature_valid, label_usable, "test")
    counts = {
        name: {
            "n": int(mask.sum()),
            "positives": int(np.nansum(labels[mask] == 1)) if mask.any() else 0,
            "positive_rate": float(np.mean(labels[mask])) if mask.any() else None,
        }
        for name, mask in (("train", train), ("val", val), ("test", test), ("gap", split == "gap"))
    }
    blocked = None
    if train.sum() < 50 or val.sum() < 20 or test.sum() < 20:
        blocked = "too few labeled samples in a chronological block"
    elif counts["train"]["positives"] in (0, counts["train"]["n"]) or len(np.unique(labels[train])) < 2:
        blocked = "training block has a single class"
    if blocked:
        return {"blocked": blocked, "counts": counts, "split_counts": {key: int((split == key).sum()) for key in ("train", "val", "test", "gap")}}

    normalizer = fit_normalizer(features, train)
    x = apply_normalizer(features, normalizer)
    y = labels
    w = np.clip(quality_weight, 0.0, None)
    w[~np.isfinite(w)] = 0.0

    analytical = analytical_scores_with_train_scale(features, feature_valid, train)
    reports = {}
    for name, scores in (
        ("analytical_motion", analytical["motion_probability"]),
        ("analytical_motion_or_vertical", analytical["motion_or_vertical_probability"]),
    ):
        reports[name] = {
            "val": _classification_report(y, scores, val, time_s, 0.5),
            "test": _classification_report(y, scores, test, time_s, 0.5),
        }

    logistic = fit_logistic(x[train], y[train], w[train], seed=seed)
    logistic_scores = predict_logistic(logistic, x)
    val_threshold = threshold_for_false_alarms(
        y, logistic_scores, _duration(time_s, val), false_alarm_target_per_minute, val
    )
    reports["logistic"] = {
        "kind": logistic["kind"],
        "val_at_0_5": _classification_report(y, logistic_scores, val, time_s, 0.5),
        "test_at_0_5": _classification_report(y, logistic_scores, test, time_s, 0.5),
        "validation_threshold_for_1_false_alarm_per_minute": val_threshold,
    }
    if val_threshold is not None:
        reports["logistic"]["test_at_validation_threshold"] = _classification_report(
            y, logistic_scores, test, time_s, val_threshold
        )

    mlp_blocked = None
    if val.sum() >= 20:
        mlp = fit_mlp(x[train], y[train], w[train], x[val], y[val], w[val], seed=seed)
        mlp_scores = predict_mlp(mlp, x)
        reports["mlp"] = {
            "val_brier_at_selection": mlp.get("val_brier"),
            "val": _classification_report(y, mlp_scores, val, time_s, 0.5),
            "test": _classification_report(y, mlp_scores, test, time_s, 0.5),
        }
    else:
        mlp_blocked = "validation block too small for early stopping"

    xgb = fit_xgboost(x[train], y[train], w[train], x[val], y[val], seed=seed)
    if xgb is None or not xgb.get("available", False):
        reports["xgboost"] = {"available": False, "reason": None if xgb is None else xgb.get("reason")}
    else:
        xgb_scores = predict_xgboost(xgb, x)
        reports["xgboost"] = {
            "available": True,
            "best_iteration": xgb.get("best_iteration"),
            "val": _classification_report(y, xgb_scores, val, time_s, 0.5),
            "test": _classification_report(y, xgb_scores, test, time_s, 0.5),
        }

    lagged = _lag_stack(x, (1, 2, 5, 10))
    lag_valid = feature_valid & lagged["valid"]
    lag_train = _rows(split, lag_valid, label_usable, "train")
    lag_val = _rows(split, lag_valid, label_usable, "val")
    lag_test = _rows(split, lag_valid, label_usable, "test")
    reports["lagged_logistic"] = {"lags": [1, 2, 5, 10], "note": "short causal context, not a TCN or GRU"}
    if lag_train.sum() > 50 and len(np.unique(y[lag_train])) > 1:
        lag_norm = fit_normalizer(lagged["features"], lag_train)
        # The base features are already normalized. Lag columns are past values
        # of those normalized features, so a second normalizer is fit on train.
        lx = apply_normalizer(lagged["features"], lag_norm)
        lag_model = fit_logistic(lx[lag_train], y[lag_train], w[lag_train], seed=seed)
        lag_scores = predict_logistic(lag_model, lx)
        reports["lagged_logistic"].update(
            {
                "val": _classification_report(y, lag_scores, lag_val, time_s, 0.5),
                "test": _classification_report(y, lag_scores, lag_test, time_s, 0.5),
            }
        )
    else:
        reports["lagged_logistic"]["blocked"] = "lagged training block is not usable"

    ablation = {}
    name_to_idx = {name: i for i, name in enumerate(FEATURE_NAMES)}
    for group, members in FEATURE_GROUPS.items():
        drop = [name_to_idx[member] for member in members]
        keep = [i for i in range(len(FEATURE_NAMES)) if i not in drop]
        model = fit_logistic(x[train][:, keep], y[train], w[train], seed=seed)
        scores = predict_logistic(model, x[:, keep])
        ablation[group] = {
            "dropped": list(members),
            "val_pr_auc": average_precision(y, scores, val),
            "val_roc_auc": roc_auc(y, scores, val),
        }
    reports["ablation_validation_only"] = ablation
    reports["mlp_blocked"] = mlp_blocked
    full_val = reports["logistic"]["val_at_0_5"]["pr_auc"]
    analytical_val = reports["analytical_motion"]["val"]["pr_auc"]
    xgb_val = reports["xgboost"].get("val", {}).get("pr_auc") if reports["xgboost"].get("available") else None
    return {
        "blocked": None,
        "counts": counts,
        "split_counts": {key: int((split == key).sum()) for key in ("train", "val", "test", "gap")},
        "reports": reports,
        "decision": {
            "validation_pr_auc": {
                "analytical_motion": analytical_val,
                "logistic": full_val,
                "xgboost": xgb_val,
                "mlp": None if "mlp" not in reports else reports["mlp"]["val"]["pr_auc"],
                "lagged_logistic": reports["lagged_logistic"].get("val", {}).get("pr_auc"),
            },
            "learned_model_beats_analytical_on_validation": _beats(full_val, analytical_val) or _beats(xgb_val, analytical_val),
            "temporal_context_beats_logistic_on_validation": _beats(
                reports["lagged_logistic"].get("val", {}).get("pr_auc"), full_val
            ),
        },
    }


def _beats(candidate, baseline, margin: float = 0.02) -> bool | None:
    if candidate is None or baseline is None:
        return None
    return bool(candidate > baseline + margin)


def _lag_stack(features: np.ndarray, lags: tuple[int, ...]) -> dict:
    n, dim = features.shape
    blocks = [features]
    valid = np.ones(n, dtype=bool)
    for lag in lags:
        shifted = np.vstack([np.zeros((lag, dim)), features[:-lag]])
        valid[:lag] = False
        valid &= np.isfinite(shifted).all(axis=1)
        blocks.append(shifted)
    stacked = np.hstack(blocks)
    stacked[~np.isfinite(stacked)] = 0.0
    return {"features": stacked, "valid": valid}
