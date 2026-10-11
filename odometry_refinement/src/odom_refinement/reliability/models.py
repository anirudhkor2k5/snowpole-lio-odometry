"""Small supervised reliability models.

Normalization statistics and early stopping use the training block and the
chronological validation block. The test block is not used to fit scales,
choose epochs, or choose hyperparameters.
"""

from __future__ import annotations

import numpy as np


def chronological_split(
    time_s: np.ndarray,
    *,
    train_frac: float = 0.5,
    val_frac: float = 0.2,
    gap_s: float = 20.0,
) -> np.ndarray:
    """Label each sample train, val, test, or gap.

    Gaps sit after the train block and after the validation block so a label
    that looks a few seconds ahead does not land in the next block.
    """
    time_s = np.asarray(time_s, dtype=np.float64)
    start = float(time_s[0])
    span = float(time_s[-1] - time_s[0])
    train_end = start + train_frac * span
    val_end = train_end + val_frac * span
    split = np.full(len(time_s), "gap", dtype=object)
    split[time_s < train_end] = "train"
    split[(time_s >= train_end + gap_s) & (time_s < val_end)] = "val"
    split[time_s >= val_end + gap_s] = "test"
    return split


def fit_normalizer(features: np.ndarray, mask: np.ndarray) -> dict:
    data = np.asarray(features, dtype=np.float64)
    chosen = data[mask]
    if len(chosen) == 0:
        raise ValueError("normalizer has no training rows")
    mean = np.nanmean(chosen, axis=0)
    std = np.nanstd(chosen, axis=0)
    std[~np.isfinite(std) | (std < 1e-8)] = 1.0
    mean[~np.isfinite(mean)] = 0.0
    return {"mean": mean, "std": std}


def apply_normalizer(features: np.ndarray, normalizer: dict) -> np.ndarray:
    data = np.asarray(features, dtype=np.float64).copy()
    data[~np.isfinite(data)] = 0.0
    return (data - normalizer["mean"]) / normalizer["std"]


def _sigmoid(z: np.ndarray) -> np.ndarray:
    z = np.clip(z, -30, 30)
    return 1.0 / (1.0 + np.exp(-z))


def fit_logistic(
    features: np.ndarray,
    labels: np.ndarray,
    weights: np.ndarray,
    *,
    l2: float = 1.0,
    steps: int = 400,
    lr: float = 0.2,
    seed: int = 0,
) -> dict:
    """L2-regularized logistic regression with balanced class weights."""
    try:
        from sklearn.linear_model import LogisticRegression

        clf = LogisticRegression(
            C=1.0 / l2,
            max_iter=500,
            class_weight="balanced",
            solver="lbfgs",
            random_state=seed,
        )
        clf.fit(features, labels, sample_weight=weights)
        return {"kind": "sklearn_logistic", "model": clf, "coef": clf.coef_.ravel(), "bias": float(clf.intercept_[0])}
    except Exception:
        pass
    x = np.asarray(features, dtype=np.float64)
    y = np.asarray(labels, dtype=np.float64)
    w = np.asarray(weights, dtype=np.float64)
    n_pos = max(float((y == 1).sum()), 1.0)
    n_neg = max(float((y == 0).sum()), 1.0)
    class_w = np.where(y == 1, 0.5 * len(y) / n_pos, 0.5 * len(y) / n_neg)
    w = w * class_w
    w = w / np.mean(w)
    rng = np.random.default_rng(seed)
    dim = x.shape[1]
    coef = rng.normal(0.0, 0.01, size=dim)
    bias = 0.0
    for _ in range(steps):
        prob = _sigmoid(x @ coef + bias)
        residual = (prob - y) * w
        grad = (x.T @ residual) / len(y) + l2 * coef
        gbias = float(residual.mean())
        coef -= lr * grad
        bias -= lr * gbias
    return {"coef": coef, "bias": bias, "kind": "logistic"}


def predict_logistic(model: dict, features: np.ndarray) -> np.ndarray:
    if model.get("kind") == "sklearn_logistic":
        return model["model"].predict_proba(features)[:, 1]
    return _sigmoid(features @ model["coef"] + model["bias"])


def fit_mlp(
    features: np.ndarray,
    labels: np.ndarray,
    weights: np.ndarray,
    val_features: np.ndarray,
    val_labels: np.ndarray,
    val_weights: np.ndarray,
    *,
    hidden: int = 16,
    l2: float = 1e-2,
    lr: float = 0.05,
    max_epochs: int = 40,
    patience: int = 6,
    seed: int = 0,
) -> dict:
    """One-hidden-layer network. Epochs stop on validation Brier score."""
    x = np.asarray(features, dtype=np.float64)
    y = np.asarray(labels, dtype=np.float64).reshape(-1, 1)
    w = np.asarray(weights, dtype=np.float64).reshape(-1, 1)
    n_pos = max(float((y == 1).sum()), 1.0)
    n_neg = max(float((y == 0).sum()), 1.0)
    class_w = np.where(y == 1, 0.5 * len(y) / n_pos, 0.5 * len(y) / n_neg)
    w = w * class_w
    w = w / np.mean(w)
    rng = np.random.default_rng(seed)
    dim = x.shape[1]
    w1 = rng.normal(0.0, 1.0 / np.sqrt(dim), size=(dim, hidden))
    b1 = np.zeros(hidden)
    w2 = rng.normal(0.0, 1.0 / np.sqrt(hidden), size=(hidden, 1))
    b2 = np.zeros(1)
    best = None
    best_score = np.inf
    stall = 0

    def brier(pred, target, sample_w) -> float:
        return float(np.average((pred.ravel() - target.ravel()) ** 2, weights=sample_w.ravel()))

    for _ in range(max_epochs):
        order = rng.permutation(len(x))
        for start in range(0, len(x), 128):
            batch = order[start : start + 128]
            xb = x[batch]
            yb = y[batch]
            wb = w[batch]
            hidden_pre = xb @ w1 + b1
            act = np.maximum(hidden_pre, 0.0)
            prob = _sigmoid(act @ w2 + b2)
            grad = (prob - yb) * wb / len(batch)
            dw2 = act.T @ grad + l2 * w2
            db2 = grad.sum(axis=0)
            back = (grad @ w2.T) * (hidden_pre > 0)
            dw1 = xb.T @ back + l2 * w1
            db1 = back.sum(axis=0)
            w2 -= lr * dw2
            b2 -= lr * db2
            w1 -= lr * dw1
            b1 -= lr * db1
        val_prob = predict_mlp({"w1": w1, "b1": b1, "w2": w2, "b2": b2}, val_features)
        score = brier(val_prob.reshape(-1, 1), val_labels.reshape(-1, 1), val_weights.reshape(-1, 1))
        if score < best_score - 1e-5:
            best_score = score
            best = {"w1": w1.copy(), "b1": b1.copy(), "w2": w2.copy(), "b2": b2.copy(), "kind": "mlp", "val_brier": score}
            stall = 0
        else:
            stall += 1
            if stall >= patience:
                break
    if best is None:
        best = {"w1": w1, "b1": b1, "w2": w2, "b2": b2, "kind": "mlp", "val_brier": None}
    return best


def predict_mlp(model: dict, features: np.ndarray) -> np.ndarray:
    act = np.maximum(features @ model["w1"] + model["b1"], 0.0)
    return _sigmoid(act @ model["w2"] + model["b2"]).ravel()


def fit_xgboost(features, labels, weights, val_features, val_labels, seed: int = 0) -> dict | None:
    try:
        import xgboost as xgb
    except Exception as exc:
        return {"available": False, "reason": str(exc)}
    model = xgb.XGBClassifier(
        n_estimators=80,
        max_depth=2,
        learning_rate=0.08,
        subsample=0.8,
        colsample_bytree=0.8,
        reg_lambda=2.0,
        min_child_weight=5.0,
        objective="binary:logistic",
        eval_metric="logloss",
        early_stopping_rounds=10,
        random_state=seed,
        n_jobs=1,
    )
    model.fit(
        features,
        labels,
        sample_weight=weights,
        eval_set=[(val_features, val_labels)],
        verbose=False,
    )
    return {"available": True, "model": model, "kind": "xgboost", "best_iteration": int(getattr(model, "best_iteration", 0))}


def predict_xgboost(model: dict, features: np.ndarray) -> np.ndarray:
    return model["model"].predict_proba(features)[:, 1]
