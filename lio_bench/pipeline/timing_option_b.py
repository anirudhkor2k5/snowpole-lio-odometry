"""Runtime of the option B runs, from what run_ros.py recorded (no re-runs). Appends to results/optionB_all/SCORES.md.

  python lio_bench/pipeline/timing_option_b.py

Per run: CPU (100 % = one core) and peak RAM of each process, sampled during the real-time replay;
latency = wall time from a scan arriving to the pose for that scan being received; KISS-ICP time per
scan inside the node (node.log). Laptop (Apple M-series, Asahi Linux): indicative only, the paper's
runtime table should come from the target computer.
"""
import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from liobench.concurrency import _latency, _resources  # noqa: E402
from liobench.io import run_dir  # noqa: E402

MD = Path(__file__).resolve().parents[1] / "results" / "optionB_all" / "SCORES.md"
RUNS = [("Faster-LIO", "faster_lio", "rerun_b", None, "solo run (raw / gravity / gk rows)"),
        ("Faster-LIO", "faster_lio", "rerun_b_fusion", "kiss_fusion", "with its node (node row)"),
        ("FAST-LIO2", "fast_lio", "optb", "kiss_fusion_fast_lio", "with its node"),
        ("DLIO", "dlio", "optb", "kiss_fusion_dlio", "with its node (100 Hz poses)"),
        ("GenZ-LIO", "genz_lio", "optb", "kiss_fusion_genz_lio", "with its node")]


def node_stats(algo, tag):
    m = re.search(r"KISS_FUSION_STATS (\{.*\})", (run_dir(algo, tag) / "node.log").read_text())
    return json.loads(m.group(1)) if m else {}


def f(x, n=0):
    return "—" if x is None else f"{x:.{n}f}"


def main():
    L = ["\n## Runtime (real-time replay on the laptop, indicative only)\n",
         "| LIO | run | LIO CPU % | LIO RAM MB | LIO latency median / p95 ms | node CPU % | node RAM MB | "
         "node latency median / p95 ms | KISS-ICP in node, ms/scan median / p95 | scans KISS-ICP processed |",
         "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for label, algo, tag, node, desc in RUNS:
        r, l = _resources(algo, tag), _latency(algo, tag)
        lat = f"{f(l.get('latency_ms_median'))} / {f(l.get('latency_ms_p95'))}" if "latency_ms_median" in l else "n/a (100 Hz timer poses)"
        if node:
            rn, ln, s = _resources(node, tag), _latency(node, tag), node_stats(node, tag)
            cells = [f(rn.get("cpu_pct_mean")), f(rn.get("rss_mb_max")),
                     f"{f(ln.get('latency_ms_median'))} / {f(ln.get('latency_ms_p95'))}",
                     f"{f(s.get('kiss_ms_median'))} / {f(s.get('kiss_ms_p95'))}", f"{s.get('scans', '—')} / 5419"]
        else:
            cells = ["—"] * 5
        L.append(f"| {label} | {desc} | {f(r.get('cpu_pct_mean'))} | {f(r.get('rss_mb_max'))} | {lat} | " + " | ".join(cells) + " |")
    kiss = json.loads((run_dir("kiss_icp_vox20", "optb") / "meta.json").read_text())
    L += ["",
          f"KISS-ICP alone (offline pass, 2.0 m, 4 threads): {kiss['ms_per_scan']:.0f} ms/scan, "
          f"{kiss['wall_s'] / 60:.1f} min for the whole drive (the drive is 9.0 min, so faster than real time).",
          "The offline gravity and gravity + KISS rows are post-processing of recorded runs and take seconds.",
          "Every run replayed the bag at real-time speed (~9.1 min of playback per run). DLIO publishes poses from a "
          "100 Hz timer, so its per-scan latency is not defined; its node's latency is measured the same way as the others "
          "(pose received after the latest scan)."]
    text = MD.read_text()
    text = text.split("\n## Runtime")[0].rstrip("\n") + "\n" + "\n".join(L) + "\n"
    MD.write_text(text)
    print("\n".join(L))


if __name__ == "__main__":
    main()
