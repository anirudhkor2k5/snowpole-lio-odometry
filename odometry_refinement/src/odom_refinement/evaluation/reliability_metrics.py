"""Ranking and calibration metrics for reliability scores.

Positive labels are 1. Scores are probabilities in [0, 1] when calibration
metrics are reported. Higher scores rank more likely failures.
"""

from __future__ import annotations

import numpy as np


def _as_float_arrays(y_true: np.ndarray, scores: np.ndarray, mask: np.ndarray | None = None):
    y = np.asarray(y_true, dtype=np.float64)
    s = np.asarray(scores, dtype=np.float64)
    keep = np.isfinite(y) & np.isfinite(s)
    if mask is not None:
        keep &= np.asarray(mask, dtype=bool)
    return y[keep], s[keep]


def roc_auc(y_true: np.ndarray, scores: np.ndarray, mask: np.ndarray | None = None) -> float | None:
    y, s = _as_float_arrays(y_true, scores, mask)
    n_pos = float((y == 1).sum())
    n_neg = float((y == 0).sum())
    if n_pos == 0 or n_neg == 0:
        return None
    order = np.argsort(s)
    ranks = np.empty(len(s), dtype=np.float64)
    ranks[order] = np.arange(1, len(s) + 1, dtype=np.float64)
    # Average ranks for ties.
    sorted_s = s[order]
    start = 0
    while start < len(s):
        stop = start + 1
        while stop < len(s) and sorted_s[stop] == sorted_s[start]:
            stop += 1
        if stop > start + 1:
            average = 0.5 * (start + 1 + stop)
            ranks[order[start:stop]] = average
        start = stop
    sum_pos = ranks[y == 1].sum()
    return float((sum_pos - n_pos * (n_pos + 1) / 2.0) / (n_pos * n_neg))


def average_precision(y_true: np.ndarray, scores: np.ndarray, mask: np.ndarray | None = None) -> float | None:
    y, s = _as_float_arrays(y_true, scores, mask)
    if len(y) == 0 or (y == 1).sum() == 0:
        return None
    order = np.argsort(-s)
    y = y[order]
    tp = np.cumsum(y == 1)
    fp = np.cumsum(y == 0)
    precision = tp / np.maximum(tp + fp, 1)
    recall_prev = np.concatenate([[0.0], tp[:-1] / tp[-1]])
    recall = tp / tp[-1]
    return float(np.sum((recall - recall_prev) * precision))


def binary_counts(y_true: np.ndarray, scores: np.ndarray, threshold: float, mask: np.ndarray | None = None) -> dict:
    y, s = _as_float_arrays(y_true, scores, mask)
    pred = s >= threshold
    tp = int(((pred == 1) & (y == 1)).sum())
    fp = int(((pred == 1) & (y == 0)).sum())
    tn = int(((pred == 0) & (y == 0)).sum())
    fn = int(((pred == 0) & (y == 1)).sum())
    precision = tp / (tp + fp) if (tp + fp) else None
    recall = tp / (tp + fn) if (tp + fn) else None
    return {"tp": tp, "fp": fp, "tn": tn, "fn": fn, "precision": precision, "recall": recall, "n": int(len(y))}


def false_alarms_per_minute(fp: int, duration_s: float) -> float | None:
    if duration_s <= 0:
        return None
    return float(fp / (duration_s / 60.0))


def brier_score(y_true: np.ndarray, probabilities: np.ndarray, mask: np.ndarray | None = None) -> float | None:
    y, p = _as_float_arrays(y_true, probabilities, mask)
    if len(y) == 0:
        return None
    p = np.clip(p, 0.0, 1.0)
    return float(np.mean((p - y) ** 2))


def negative_log_likelihood(y_true: np.ndarray, probabilities: np.ndarray, mask: np.ndarray | None = None) -> float | None:
    y, p = _as_float_arrays(y_true, probabilities, mask)
    if len(y) == 0:
        return None
    p = np.clip(p, 1e-6, 1.0 - 1e-6)
    return float(-np.mean(y * np.log(p) + (1.0 - y) * np.log(1.0 - p)))


def expected_calibration_error(
    y_true: np.ndarray,
    probabilities: np.ndarray,
    mask: np.ndarray | None = None,
    n_bins: int = 10,
) -> dict:
    y, p = _as_float_arrays(y_true, probabilities, mask)
    if len(y) == 0:
        return {"ece": None, "bins": []}
    p = np.clip(p, 0.0, 1.0)
    edges = np.linspace(0.0, 1.0, n_bins + 1)
    ece = 0.0
    bins = []
    for start, stop in zip(edges[:-1], edges[1:]):
        if stop == 1.0:
            chosen = (p >= start) & (p <= stop)
        else:
            chosen = (p >= start) & (p < stop)
        if chosen.sum() == 0:
            continue
        confidence = float(p[chosen].mean())
        observed = float(y[chosen].mean())
        weight = float(chosen.mean())
        ece += weight * abs(confidence - observed)
        bins.append(
            {
                "start": float(start),
                "stop": float(stop),
                "n": int(chosen.sum()),
                "confidence": confidence,
                "observed_frequency": observed,
            }
        )
    return {"ece": float(ece), "bins": bins}


def threshold_for_false_alarms(
    y_true: np.ndarray,
    scores: np.ndarray,
    duration_s: float,
    target_per_minute: float,
    mask: np.ndarray | None = None,
) -> float | None:
    """Highest threshold whose false-alarm rate is still at or below the target.

    Ties keep the more conservative (higher) threshold. Returns None when the
    negative class is absent.
    """
    y, s = _as_float_arrays(y_true, scores, mask)
    if (y == 0).sum() == 0 or duration_s <= 0:
        return None
    order = np.argsort(-s)
    y = y[order]
    s = s[order]
    best = None
    fp = 0
    for i, (label, score) in enumerate(zip(y, s)):
        if label == 0:
            fp += 1
        rate = fp / (duration_s / 60.0)
        # Threshold just below this score would include it. Use this score as
        # the operating point that includes the current prefix.
        if rate <= target_per_minute:
            best = float(score)
        else:
            break
    return best
