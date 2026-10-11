# lio_bench — Report v2: Phase A audit

*2026-10-11. Odometry only (pole localisation and full-backend sliding windows are out of scope).
Exact methods and the leakage audit: `METHODS.md`. Raw numbers: `results/audit/audit.json`.
v1 (everything before this): `STUDY_SUMMARY.md`, `results/REPORT.md`. Nothing committed to git.*

---

## 0. Executive summary

1. **The dominant error is heading drift, not tilt and not scale.** Under a deployable
   protocol (one alignment on the first 200 m, then pure prediction), about 97 % of
   Faster-LIO's 10 km error is heading. Its heading drifts +32.6° by km 10; given the true
   heading, the same odometry stays within **30 m** (ATE 1 245 → 30 m). The same holds for
   all five methods (25–32 m with the true heading).
2. **Heading drifts at a near-constant rate per second, in the same direction for every
   method** (counter-clockwise): Faster-LIO 0.06°/s, FAST-LIO 0.047, GenZ-LIO 0.044, DLIO
   0.016–0.038, KISS-ICP 0.02 (no IMU). That signature points to a bias (gyro bias, or a
   fixed per-scan rotation), not to turns or degenerate scenes. Earth's rotation explains
   only 0.0037°/s. Without an absolute heading reference this bias is unobservable, so no
   local ICP gating can remove it.
3. **The tilt failure is pitch**, and the mechanism is now confirmed. Removing pitch alone
   restores the track length; removing roll alone does nothing. Locking roll/pitch to IMU
   gravity turns the raw VGICP corrector from 8.40 % drift / 0.916 length into 1.37 % /
   0.998, online, with no post-processing. A gravity-only online filter (no ICP) matches
   the offline planar fix.
4. **The ground truth is the vehicle's own dual-antenna GNSS** (RTK float, antenna baseline
   stable to 1.3 cm). It is fine for metre-level drift, but per-step GT yaw is
   noise-dominated, which matters for the fusion model's yaw labels.
5. **Re-scored with the canonical protocol, some v1 claims change.** On the test block,
   KISS-ICP (7.9 m), baseline + wheel (7.6–8.4 m) and DLIO (9.7 m) are as good as or better
   than the learned fusion (8.5–14.3 m) in ATE. The fusion's advantage is in **local** drift
   (0.41–0.47 % vs 0.59–0.94 %), not in where the car ends up. Confidence intervals overlap
   for most top methods.
6. **Implication for the research plan:** "when is a correction safe" (observability gating)
   addresses a secondary error. The primary error is a slow heading bias. Gate design should
   come after choosing **the lowest-bias heading source**, and pitch/roll must come from
   gravity, never from ICP.

---

## 1. What was done in Phase A

| task | done | output |
|---|---|---|
| RAM | freed 1.7 GB of RAM-backed `/tmp` (my leftover repo clones, an unused 935 MB venv, logs); no user processes touched | `/tmp` 1.7 GB → 59 MB |
| A1 ground-truth provenance | ✅ | §2 |
| A2 canonical protocol + error decomposition | ✅ `audit.canonical`, `audit.decompose` | §3 |
| A3 planar ablation | ✅ 7 variants × 2 baselines | §4 |
| A4 mechanism test: gravity-locked VGICP | ✅ 2 full-drive passes (≈2× real time, 3 GB cap) | §5 |
| heading-drift attribution | ✅ bias vs scale vs scene; per second vs per metre | §6 |
| A5 `METHODS.md` | ✅ equations + leakage audit | file |
| A6 bugs | ✅ GenZ warm start was non-causal by 0.3 s (fixed: backward difference); window compute read 0 s for ROS supports (fixed: CPU-seconds from resource logs) | §8 |
| re-score v1 results canonically | ✅ correctors + fusion on the test block, with 95 % CIs | §7 |

---

## 2. A1 — ground truth provenance

| check | result | consequence |
|---|---|---|
| what it is | dual-antenna GNSS midpoint (GT − midpoint: median 3.4 cm, p95 12 cm); lat/lon only, no covariance | not an independent survey; any GNSS lever-arm or timing error is shared with the reference |
| fix quality | RTK float (status 5) on 5 382 samples, dead reckoning (6) on 40; NavSat status GBAS | decimetre-class absolute accuracy |
| antenna baseline | 0.499 m, std 1.3 cm (p5–p95 0.493–0.503 m) | relative precision ~1 cm; receivers healthy |
| GT stamps | GNSS header stamps (receiver − header: median 0.7 ms) | timing consistent |
| GT jitter vs 1 s moving average | median 7 cm, p95 22 cm (upper bound: includes curvature) | **per-step labels have cm-level noise**; at 2 m steps GT yaw has ~2° per-step noise |

**Verdict:** valid for everything measured in metres over hundreds of metres. Not valid for
centimetre claims. Per-step yaw comparisons against GT are noise-limited: regressing every
method's per-step yaw error on GT yaw gives the same spurious −31 % "scale". That is noise
in the GT yaw, not a real scale error.

---

## 3. A2 — canonical protocol and error budget

**Protocol (used for every number below):**
- clock residual estimated on the training block only;
- one SE(2) alignment on the first 200 m of the evaluated span;
- after that, pure prediction.

Drift = KITTI-style 100–800 m segment drift. 95 % CIs by moving-block bootstrap.

**Error budget:** counterfactual re-integration, whole drive, ATE in m.

| method | as is | tilt removed (planar) | + true heading | + true forward length | heading at km 10 | heading rate | forward length ratio |
|---|---:|---:|---:|---:|---:|---:|---:|
| Faster-LIO | 1 245 | 917 | **29.7** | 25.3 | +32.6° | 3.1°/km | 0.9965 |
| FAST-LIO | 1 486 | 749 | **31.6** | 27.3 | +25.1° | 2.4°/km | 0.9977 |
| DLIO | 1 103 | 573 | **25.4** | 25.8 | +20.1° | 1.8°/km | 1.0011 |
| GenZ-LIO | 1 409 | 798 | **29.1** | 27.4 | +23.1° | 2.1°/km | 0.9987 |
| KISS-ICP | 563 | 372 | **27.9** | 25.2 | +12.6° | 0.95°/km | 0.9957 |

**Reading it:**
- **Tilt** costs 30–50 % of the error.
- **Heading** is almost all of the rest.
- **Forward scale** is right to 0.1–0.4 % and costs at most 4 m.
- **What remains with true heading and length** (25–27 m) is lateral motion plus noise, and it is the same for every method.

On the 2 km test block the same pattern holds on a smaller scale. Faster-LIO: 131 → 27 → 9.8 → 2.7 m.

---

## 4. A3 — what exactly "planar" fixes

Faster-LIO / FAST-LIO, whole drive and test block (canonical):

| variant | whole ATE | whole drift | length | heading at end | test drift 200 m [95 % CI] |
|---|---:|---:|---:|---:|---|
| as is | 1 245 / 1 486 | 5.05 / 9.40 % | 0.953 / 0.905 | +29.5 / +19.1° | 13.3 [8.4–18.3] / 25.6 [14.4–36.7] % |
| **remove pitch only** | 1 223 / 1 432 | **1.90 / 2.76 %** | **0.997 / 0.998** | +29.4 / +19.1° | 1.8 / 3.8 % |
| remove roll only | 1 225 / 1 459 | 5.14 / 9.63 % | 0.952 / 0.903 | +30.0 / +20.3° | 13.4 / 26.0 % |
| level, yaw from the tilted world | 1 202 / 1 405 | 1.89 / 2.74 % | 0.997 / 0.998 | +29.8 / +19.8° | 1.8 / 3.7 % |
| **planar** (body-frame yaw about measured up) | **948 / 782** | **1.39 / 1.20 %** | 0.997 / 0.998 | +29.8 / +22.4° | **0.94 [0.47–1.90] / 0.65 [0.44–1.10] %** |
| planar, up = IMU z | 1 032 / 898 | 1.46 / 1.28 % | 0.997 / 0.998 | +32.5 / +25.9° | 0.97 / 0.68 % |
| planar, no lateral component | 929 / 759 | 1.13 / 0.94 % | 0.997 / 0.998 | +29.7 / +22.4° | 1.00 / 0.68 % |

**Attribution:**
1. **Pitch is the whole length problem.** Roll removal does nothing.
2. **Integrating yaw about the body's vertical** gains another 20–45 % of ATE over levelling the
   world pose. A drifting world attitude contaminates the world yaw angle.
3. **The measured vertical axis (1.7° from IMU z) matters:** it is worth 8–13 % of ATE.
4. **Dropping lateral motion** (the vehicle constraint) lowers drift a little. It is not the main effect.
5. **Planar never fixes heading drift:** the end heading error is unchanged at about +20–30°.

---

## 5. A4 — mechanism test: gravity-locked correction (online, no post-processing)

The corrector registers with VGICP. Roll and pitch are pulled toward the IMU's gravity
direction with a complementary filter (time constant 5 s). The vehicle's own acceleration is
subtracted using only the IMU and the baseline's speed, so this is LiDAR+IMU only.
`grav_only` = the same attitude fix with no ICP. Snapping attitude every scan made ICP fail
on 54 % of scans (accelerometer jitter blurs the map); the 5 s filter: 0.5 %.

| Faster-LIO baseline | whole ATE | whole drift | length | heading | val drift 200 m | test drift 200 m [CI] | test ATE |
|---|---:|---:|---:|---:|---:|---|---:|
| Faster-LIO | 1 245 | 5.05 % | 0.953 | +29.5° | — | 13.3 % | 131 |
| VGICP, raw | 1 242 | **8.40 %** | **0.916** | +18.3° | 7.90 % | 23.9 [14.3–33.9] % | 228 |
| **VGICP, gravity-locked** | **711** | **1.37 %** | **0.998** | +25.5° | 0.62 % | 0.72 [0.52–0.99] % | 36 |
| **gravity only (no ICP)** | 965 | 1.43 % | 0.996 | +29.4° | 0.84 % | 0.92 [0.45–1.88] % | 23 |
| planar (offline) | 948 | 1.39 % | 0.997 | +29.8° | 0.84 % | 0.94 [0.47–1.90] % | 27 |
| VGICP + planar (offline) | **655** | **1.15 %** | 1.001 | +21.5° | 0.62 % | **0.48 [0.40–0.57] %** | **15** |

FAST-LIO baseline: same picture. Gravity-locked VGICP 729 m / 1.37 %; gravity-only 818 m /
1.27 %; VGICP + planar 671 m / 1.15 %.

**Conclusions:**
- **Confirmed: ICP "harm" is attitude drift.** Scan-to-map registration has no gravity
  reference. Fix its roll/pitch from gravity and the harm disappears (8.40 → 1.37 %).
- **The tilt fix is deployable online.** `grav_only` ≈ offline planar.
- **Offline planar + VGICP is still the best,** because the online filter's 5 s time
  constant is untuned. It deliberately was not tuned on this drive.
- **VGICP contributes heading information:** VGICP variants end with less heading error
  (+18…+25°) than the baseline (+29.5°).

---

## 6. The real problem: heading drift

### 6.1 What it isn't

Regression of per-step yaw error on time, yaw rate and |yaw rate|, plus where it accumulates:

| method | total | constant-rate part | accumulated in weak / mid / strong scene constraint |
|---|---:|---:|---|
| Faster-LIO | +31.0° | +0.038°/s → +19.4° | +9 / +13 / +9 |
| FAST-LIO | +23.7° | +0.024°/s → +12.0° | +5 / +12 / +8 |
| DLIO | +18.0° | +0.045°/s → +23.1° | +3 / +10 / +5 |
| GenZ-LIO | +22.2° | +0.050°/s → +25.4° | +2 / +11 / +9 |
| KISS-ICP | +11.3° | +0.024°/s → +12.1° | −2 / +6 / +6 |

- **Not concentrated in degenerate (weakly constrained) scenes.** It accumulates evenly.
- **Not driven by turning.** The route's net turning is −8° (697° in absolute turns); the turn terms are ±4–8°.
- **Not scale.** The apparent −31 % "scale" is GT yaw noise (§2).

### 6.2 What it is: time-driven

| method | slow section (~10 m/s) | fast section (~21 m/s) |
|---|---:|---:|
| Faster-LIO | 0.054°/s (3.65°/km) | 0.062°/s (2.95°/km) |
| FAST-LIO | 0.044°/s (2.98°/km) | 0.047°/s (2.23°/km) |
| GenZ-LIO | 0.041°/s (2.74°/km) | 0.044°/s (2.10°/km) |
| KISS-ICP | 0.019°/s (1.26°/km) | 0.023°/s (1.08°/km) |
| DLIO | 0.016°/s | 0.038°/s |

For four of the five methods the rate per **second** stays roughly constant while the rate
per km changes 20–30 %. That is a bias signature.

### 6.3 Likely causes (not yet separated)

- **Gyro z bias leaking into yaw.** It is consistent with the IMU-coupled methods drifting
  2–3× more than KISS-ICP. ~~The gyro reads +0.003 rad/s on z while parked~~ **Erratum (v3):**
  that average included the start of motion (the car starts turning at ~6 s while CAN still
  reads 0). The true standstill bias (0–6 s) is −0.00065 rad/s, which the LIOs initialise
  correctly. The drift is carried by the *online* bias estimate wandering away from it
  (REPORT_v3 §8).
- **Earth's rotation:** +0.0037°/s at 63.2° N. None of the LIOs model it. It is a real but
  small part (~2° of 20–30°).
- **A fixed per-scan rotation error in the LiDAR pipeline** (deskew at the scan seam,
  LiDAR–IMU time offset): needed to explain KISS-ICP's 0.02°/s with no IMU at all.

**Consequence:** a constant heading bias cannot be removed by local corrections; it is
unobservable without an absolute heading. Available levers:
(a) pick or weight the lowest-bias yaw source (KISS-ICP, VGICP);
(b) remove the known Earth-rate term;
(c) find and fix the per-scan bias in the pipeline;
(d) an absolute heading reference (out of scope now).

---

## 7. v1 results re-scored with the canonical protocol (test block, ~2 km)

Fusion outputs are integrated over the test block, then evaluated identically to everything
else:

| | ATE | final | drift 200 m [95 % CI] | heading at end |
|---|---:|---:|---|---:|
| Faster-LIO (tilt-invariant) | 26.7 | 57.2 | 0.94 [0.47–1.90] % | 4.0° |
| FAST-LIO (tilt-invariant) | 16.4 | 34.0 | 0.66 [0.45–1.10] % | 1.9° |
| DLIO (tilt-invariant) | 9.7 | 26.1 | 0.68 [0.55–0.85] % | 3.2° |
| GenZ-LIO (tilt-invariant) | 15.3 | 36.4 | 0.59 [0.40–0.96] % | 3.0° |
| **KISS-ICP** (tilt-invariant) | **7.9** | **16.2** | 0.93 [0.65–1.54] % | 3.1° |
| **FAST-LIO + wheel** | **7.6** | **8.2** | 0.49 [0.42–0.58] % | 1.0° |
| Faster-LIO + wheel | 8.4 | 24.3 | 0.58 [0.44–0.69] % | 3.1° |
| VGICP + planar (Faster-LIO) | 14.7 | — | **0.48 [0.40–0.57] %** | — |
| fusion, FAST-LIO + DLIO + GenZ (MLP-16) | 11.7 | 28.1 | **0.44 [0.34–0.60] %** | 2.4° |
| fusion, Faster-LIO + DLIO + GenZ (MLP-16) | 14.3 | 33.8 | **0.44 [0.36–0.58] %** | 2.8° |
| fusion with correctors (FAST-LIO, linear) | 8.5 | 22.6 | **0.41 [0.32–0.56] %** | 2.1° |
| fusion with correctors (Faster-LIO, linear) | 21.6 | 52.8 | 0.47 [0.32–0.67] % | 4.2° |
| v1 "selected on val" corrector (plane-ICP DA + planar) | 24.4–28.6 | 64–74 | 0.91–1.17 % | 6–7° |

**What changes vs v1:**
- **Learned fusion is best at local drift** (0.41–0.47 %, CIs roughly half those of the
  baselines). On ATE it is matched or beaten by the simple options: KISS-ICP, baseline +
  wheel, DLIO. ATE over 2 km is dominated by a few degrees of heading, so ATE differences of
  a few metres here aren't reliable. v1's "−46 % for fusion" used a different protocol and
  overstates the gain.
- **DLIO is the best single LIO on this block** (9.7 m), though the training-block "best
  single" pick was GenZ-LIO. Selection on train/val is unreliable at this sample size.
- **Most confidence intervals overlap.** Only the separations of tilt-fixed vs not, and of
  local drift, fusion vs single baseline, are clearly outside the noise.

---

## 8. A6 — bugs fixed

| bug | effect | fix |
|---|---|---|
| GenZ-LIO warm start used a central difference (pose 0.3 s in the future) | slight non-causality in the warm-start windows | backward difference over the last 0.6 s (`genz_runner._baseline_body_velocity`) |
| window compute read `wall_s`, which only the offline GenZ runner writes | DLIO window compute always 0 s | CPU-seconds integrated from `resources.csv` for ROS supports (`windows.py`) |
| (from v1) window runs named without the baseline | Faster-LIO windows reused FAST-LIO's runs | fixed and rerun in v1 |

Windows are out of scope now, so they were not rerun after these fixes.

---

## 9. What this means for the plan

| original Phase | status after Phase A |
|---|---|
| B: correction-utility dataset (skip / VGICP / gated VGICP) | **still worth doing, with a different target.** VGICP's value is mostly heading information plus local drift. Utility labels should be measured over windows long enough to see heading effects (≥ 30–60 s), and attitude must always come from gravity. |
| C: direction-specific gating | **lower priority.** Errors don't concentrate in degenerate scenes. Gating pitch/roll is solved by gravity locking, and horizontal degeneracy isn't where the error is. |
| D: utility controller | after B; must beat the equal-budget periodic baseline and the "always gravity-locked VGICP" baseline (8 ms/scan, cheap enough to run always) |
| E: direct vs fixed-lag | still valid; expected to add little with only relative constraints |
| **new, highest priority: heading bias** | (1) add the Earth-rate term; (2) find KISS-ICP's per-scan yaw bias (deskew/seam/time offset); (3) a yaw-specific fusion that learns which source has the least bias, trained on **multi-second** yaw targets, not noisy per-step ones |
| **new: online gravity filter** | the deployable replacement for offline planar. It must be tuned on train/val only (time constant, acceleration compensation). |

---

## 10. Limitations

- **One 10 km drive, one ~2 km test block.** CIs come from block bootstrap within the
  drive, not from independent runs.
- **The GT is the vehicle's own GNSS** (RTK float). It is fine at metre level, noisy per step.
- **Laptop timings** (M-series performance/efficiency cores) are indicative only.
- **FAST-LIO and DLIO are not deterministic run to run.** Run-to-run spread wasn't measured
  (needs repeated runs, ideally on the cloud).
- **The gravity filter's time constant (5 s) was chosen a priori, not tuned.** The offline
  planar result is the upper bound it should approach.

## 11. Files

| | |
|---|---|
| this report | `lio_bench/REPORT_v2.md` |
| methods, equations, leakage audit | `lio_bench/METHODS.md` |
| raw audit numbers | `results/audit/audit.json` |
| new code | `src/liobench/audit.py` (canonical protocol, decomposition, ablation, provenance, fusion re-scoring); gravity-locked variants in `src/liobench/correct/scanpass.py` (`vgicp_grav`, `grav_only`) |
| new runs | `results/runs/{faster_lio,fast_lio}+{vgicp_grav,grav_only}/full/` |
| v1 | `STUDY_SUMMARY.md`, `results/REPORT.md` |
