"""liobench command line. Run `liobench -h` or `liobench <command> -h`.

Order of a full study (scripts/run_all.sh does all of it):
  prepare                       clock fit + auxiliary topics from the bag
  run-ros --algos fast_lio      baselines (ROS, real time)        [lio-ros env, via scripts/run_ros.sh]
  run-genz --scan-features      GenZ-LIO + per-scan context features (Python, faster than real time)
  run-ros --algos dlio          DLIO (ROS)
  verify                        step 1: legacy CSVs vs ground truth and vs re-runs
  analyze                       step 3: where/why the baselines go wrong
  run-ros --algos fast_lio,dlio concurrency run;  concurrency --baseline .. --support ..
  fusion --baseline fast_lio    step 4: learned gating + trigger model
  windows --support genz_lio    step 5: support in adaptive / periodic sliding windows
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys

from .io import BENCH_ROOT, load_config


def _print(obj):
    print(json.dumps(obj, indent=2, default=float)[:6000])


def main(argv=None):
    ap = argparse.ArgumentParser(prog="liobench", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("prepare", help="fit sensor clock, extract INS heading / CAN / INS state from the bag")

    p = sub.add_parser("run-ros", help="run ROS estimators (forwards to scripts/run_ros.sh)", add_help=False)
    p.add_argument("rest", nargs=argparse.REMAINDER)

    p = sub.add_parser("run-genz", help="run GenZ-LIO from the bag via its Python bindings")
    p.add_argument("--tag", default="full")
    p.add_argument("--start", type=float, default=0.0, help="seconds from bag start")
    p.add_argument("--duration", type=float, default=None)
    p.add_argument("--scan-features", action="store_true", help="also write results/features/scan_context.csv")
    p.add_argument("--warm-from", default=None, help="seed velocity from this baseline's full run")
    p.add_argument("--threads", type=int, default=None)
    p.add_argument("--time-scale", type=float, default=1.0, help="deskew probe: scale per-point times (0 = off)")
    p.add_argument("--config", default=None, help="GenZ-LIO yaml (default: configs/algorithms/genz_lio.yaml)")

    sub.add_parser("verify", help="step 1: check fastlio.csv / fasterlio.csv")

    p = sub.add_parser("quick", help="sanity numbers for one run over the span it covers (smoke tests)")
    p.add_argument("--algo", required=True)
    p.add_argument("--tags", default=None, help="comma list (default: every tag of the algo)")

    p = sub.add_parser("analyze", help="step 3: baseline weakness analysis")
    p.add_argument("--window", type=float, default=10.0)

    p = sub.add_parser("concurrency", help="solo vs simultaneous baseline + support")
    p.add_argument("--baseline", default="fast_lio")
    p.add_argument("--support", default="dlio")

    p = sub.add_parser("fusion", help="step 4: train/evaluate the gating model (+ trigger model)")
    p.add_argument("--baseline", default="fast_lio")
    p.add_argument("--supports", default="dlio,genz_lio",
                   help="comma list, or 'correctors': dlio, genz_lio, kiss_icp, <baseline>+wheel and the "
                        "correction strategy selected on validation by `liobench correct report`")
    p.add_argument("--seeds", type=int, default=3, help="seeds for the held-out model (spread is reported)")
    p.add_argument("--out-name", default=None, help="results/fusion/<out-name> (default: the baseline)")
    p.add_argument("--no-cv", action="store_true")
    p.add_argument("--epochs", type=int, default=400)
    p.add_argument("--seed", type=int, default=0)

    p = sub.add_parser("windows", help="step 5: support method in sliding windows")
    p.add_argument("--baseline", default="fast_lio")
    p.add_argument("--support", default="genz_lio", help="genz_lio (fast, offline) or dlio (ROS, real time)")
    p.add_argument("--schedule", choices=["adaptive", "periodic"], default="adaptive")
    p.add_argument("--warmup", type=float, default=5.0)
    p.add_argument("--cold", action="store_true", help="cold start (default: warm start from the baseline)")
    p.add_argument("--threshold-q", type=float, default=0.75)
    p.add_argument("--length", type=float, default=20.0)
    p.add_argument("--period", type=float, default=60.0)
    p.add_argument("--whole-drive", action="store_true", help="windows over the whole drive, not only the test block")
    p.add_argument("--force", action="store_true")

    sub.add_parser("summary", help="collect every result into results/REPORT.md")

    p = sub.add_parser("export", help="write a trajectory as the snow-pole pipeline's incremental_navigation CSV")
    p.add_argument("--source", default="faster_lio", help="estimator run (or a corrected run with --pipeline run)")
    p.add_argument("--tag", default="full")
    p.add_argument("--pipeline", default="gravity+kiss_heading", choices=["raw", "gravity", "gravity+kiss_heading", "run"])
    p.add_argument("--out", default=None)

    p = sub.add_parser("kiss-probe", help="KISS-ICP with one input change (deskew/timing/voxel) to locate yaw bias")
    p.add_argument("probe", choices=["nodeskew", "revtime", "vox05", "shift5ms", "noground", "minr10", "upperbeams",
                                          "vox15", "vox20", "range50", "mppv5", "default"])
    p.add_argument("--duration", type=float, default=None)

    p = sub.add_parser("correct", help="phase 2: correction strategies on top of a baseline")
    p.add_argument("what", choices=["wheel", "scan", "planar", "poles", "report", "all"],
                   help="wheel: CAN forward motion; scan: one bag pass (ICP variants, KISS-ICP, pole candidates); "
                        "planar: tilt-invariant re-integration of the baseline, the supports and every corrector; "
                        "poles: planar first, then pole-map fixes on --methods; report: per-block comparison; "
                        "all: in that order")
    p.add_argument("--baseline", default="fast_lio")
    p.add_argument("--tag", default="full", help="output tag")
    p.add_argument("--baseline-tag", default="full", help="which run of the baseline to correct")
    p.add_argument("--variants", default=None, help="comma list of ICP variants (default: all, see correct/scanpass.py)")
    p.add_argument("--no-kiss", action="store_true")
    p.add_argument("--no-poles", action="store_true", help="scan: skip pole-candidate detection")
    p.add_argument("--threads", type=int, default=4)
    p.add_argument("--start", type=float, default=0.0)
    p.add_argument("--duration", type=float, default=None)
    p.add_argument("--methods", default=None,
                   help="poles: trajectories to correct (default: planar baseline and planar ICP correctors)")

    a = ap.parse_args(argv)
    if a.cmd == "run-ros":
        sys.exit(subprocess.call(["bash", str(BENCH_ROOT / "scripts" / "run_ros.sh"), *a.rest]))
    cfg = load_config()

    if a.cmd == "prepare":
        from .bag import extract_aux, fit_sensor_clock
        _print(fit_sensor_clock(cfg))
        print({k: str(v) for k, v in extract_aux(cfg).items()})
    elif a.cmd == "run-genz":
        from . import genz_runner
        out = genz_runner.run(tag=a.tag, start_offset_s=a.start, duration_s=a.duration, scan_features=a.scan_features,
                              warm_start_from=a.warm_from, max_threads=a.threads, time_scale=a.time_scale,
                              config_path=a.config)
        _print(json.loads((out / "meta.json").read_text()))
    elif a.cmd == "verify":
        from . import verify
        verify.run(cfg)
        print((BENCH_ROOT / "results" / "verify" / "verify_summary.md").read_text())
    elif a.cmd == "quick":
        from .io import load_ground_truth, load_run, run_tags
        from .metrics import evaluate
        gt = load_ground_truth(cfg)
        for tag in (a.tags.split(",") if a.tags else run_tags(a.algo)):
            tr = load_run(a.algo, tag, cfg)
            if tr is None or len(tr) < 20:
                print(f"{a.algo}/{tag}: no trajectory")
                continue
            s = evaluate(tr, gt.slice_time(tr.t[0] + 1.0, tr.t[-1] - 1.0), "se2_full", cfg["evaluation"]).summary
            print(f"{a.algo}/{tag}: {tr.t[-1] - tr.t[0]:6.1f} s  ATE {s['ate_xy'].get('rmse', float('nan')):7.2f} m  "
                  f"length ratio {s['path_length_m']['ratio']:.3f}  ref {s['path_length_m']['ref']:.0f} m")
    elif a.cmd == "kiss-probe":
        from .correct import kiss_probe
        _print(kiss_probe.run(a.probe, cfg, duration_s=a.duration))
    elif a.cmd == "export":
        from . import export
        path = export.run(cfg, a.source, a.tag, a.pipeline, a.out)
        print(path)
        print(path.with_suffix(".json").read_text())
    elif a.cmd == "summary":
        from . import report_all
        print(report_all.run(cfg))
    elif a.cmd == "correct":
        from .correct import poles, report, scanpass, wheel
        from .io import run_dir
        from .correct import planar
        from .correct.report import SUPPORTS

        def planarize_all():
            root = BENCH_ROOT / "results" / "runs"
            srcs = [(a.baseline, a.tag)] + [(s_, a.tag) for s_ in SUPPORTS]
            srcs += [(d.name, a.tag) for d in sorted(root.iterdir()) if d.name.startswith(f"{a.baseline}+")
                     and not d.name.endswith(("+planar", "+poles"))]
            done = []
            for m, t in srcs:
                if (run_dir(m, t) / "odom_sensor_clock.tum").exists():
                    done.append(planar.run(cfg, m, t))
            print("planar:", done)

        steps = ["wheel", "scan", "planar", "poles", "report"] if a.what == "all" else [a.what]
        for w in steps:
            if w == "wheel":
                _print(wheel.run(cfg, a.baseline))
            elif w == "scan":
                v = [x for x in a.variants.split(",") if x] if a.variants is not None else None   # "" = none
                _print(scanpass.run(cfg, a.baseline, baseline_tag=a.baseline_tag, tag=a.tag, variants=v,
                                    kiss=not a.no_kiss, poles=not a.no_poles, threads=a.threads, start_offset_s=a.start, duration_s=a.duration))
            elif w == "planar":
                planarize_all()
            elif w == "poles":
                if a.what != "all":
                    planarize_all()          # pole fixes are applied to tilt-free trajectories
                ms = a.methods.split(",") if a.methods else [
                    m for m in (f"{a.baseline}+planar", f"{a.baseline}+gicp_da+planar", f"{a.baseline}+vgicp+planar",
                                f"{a.baseline}+gicp_da_wheel+planar")
                    if (run_dir(m, a.tag) / "odom_sensor_clock.tum").exists()]
                for m in ms:
                    r = poles.correct(cfg, m, a.tag, baseline=a.baseline)
                    print(m, "fixes", r["fixes_accepted"], "/", r["candidates_tested"], "| test, pole zone:",
                          json.dumps(r["test"]["pole_zone"], default=float)[:400])
            elif w == "report":
                report.run(cfg, a.baseline, a.tag)
                print((BENCH_ROOT / "results" / "correct" / a.baseline / "correct_summary.md").read_text())
    elif a.cmd == "analyze":
        from . import analysis
        analysis.run(cfg, a.window)
        print((BENCH_ROOT / "results" / "analysis" / "analysis_summary.md").read_text())
    elif a.cmd == "concurrency":
        from . import concurrency
        _print(concurrency.run(a.baseline, a.support, cfg))
    elif a.cmd == "fusion":
        from . import fusion
        sup = a.supports.split(",")
        if a.supports == "correctors":
            summ = json.loads((BENCH_ROOT / "results" / "correct" / a.baseline / "correct_summary.json").read_text())
            sup = list(dict.fromkeys(["dlio", "genz_lio", "kiss_icp", f"{a.baseline}+wheel", summ["selected_on_val"]]))
        fusion.run(cfg, a.baseline, sup, cv=not a.no_cv, epochs=a.epochs, seed=a.seed, n_seeds=a.seeds,
                   out_name=a.out_name)
        print((BENCH_ROOT / "results" / "fusion" / (a.out_name or a.baseline) / "fusion_summary.md").read_text())
    elif a.cmd == "windows":
        from . import windows
        r = windows.run(cfg, a.baseline, a.support, a.schedule, a.warmup, not a.cold, a.threshold_q, a.length, a.period,
                        only_test_block=not a.whole_drive, force=a.force)
        _print({k: v for k, v in r.items() if k not in ("fused", "transient", "windows")})


if __name__ == "__main__":
    main()
