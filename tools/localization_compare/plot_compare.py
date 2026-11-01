#!/usr/bin/env python3
"""Plot and summarize two localization comparison trials."""

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


def as_float(row: Dict[str, str], key: str) -> Optional[float]:
    try:
        value = row.get(key, "")
        if value == "":
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_diff(a: float, b: float) -> float:
    return wrap_angle(a - b)


def unwrap_angles(values: List[float]) -> List[float]:
    if not values:
        return []
    unwrapped = [values[0]]
    previous_raw = values[0]
    for raw in values[1:]:
        unwrapped.append(unwrapped[-1] + angle_diff(raw, previous_raw))
        previous_raw = raw
    return unwrapped


def read_json(path: Path) -> Dict:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def read_pose_csv(path: Path, extra_filter=None) -> List[Dict[str, float]]:
    if not path.exists():
        return []
    rows: List[Dict[str, float]] = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if extra_filter is not None and not extra_filter(row):
                continue
            elapsed = as_float(row, "elapsed_sec")
            x = as_float(row, "x")
            y = as_float(row, "y")
            z = as_float(row, "z")
            yaw = as_float(row, "yaw_rad")
            if elapsed is None or x is None or y is None or z is None or yaw is None:
                continue
            rows.append(
                {
                    "elapsed": elapsed,
                    "x": x,
                    "y": y,
                    "z": z,
                    "yaw": yaw,
                }
            )
    rows.sort(key=lambda item: item["elapsed"])
    return rows


def read_tf_rows(run_dir: Path, target: str, source: str) -> List[Dict[str, float]]:
    return read_pose_csv(
        run_dir / "tf_samples.csv",
        lambda row: row.get("target_frame") == target and row.get("source_frame") == source,
    )


def read_raw_topic_rows(run_dir: Path, topic: str) -> List[Dict[str, float]]:
    return read_pose_csv(run_dir / "raw_pose_samples.csv", lambda row: row.get("topic") == topic)


def normalize_time(rows: List[Dict[str, float]], drive_start: float) -> List[Dict[str, float]]:
    normalized = []
    for row in rows:
        copied = dict(row)
        copied["t"] = row["elapsed"] - drive_start
        normalized.append(copied)
    return normalized


def step_series(rows: List[Dict[str, float]]) -> Tuple[List[float], List[float], List[float]]:
    times: List[float] = []
    translation_steps: List[float] = []
    yaw_steps_deg: List[float] = []
    for prev, cur in zip(rows, rows[1:]):
        dx = cur["x"] - prev["x"]
        dy = cur["y"] - prev["y"]
        dz = cur["z"] - prev["z"]
        times.append(cur.get("t", cur["elapsed"]))
        translation_steps.append(math.sqrt(dx * dx + dy * dy + dz * dz))
        yaw_steps_deg.append(abs(math.degrees(angle_diff(cur["yaw"], prev["yaw"]))))
    return times, translation_steps, yaw_steps_deg


def mean(values: Iterable[float]) -> float:
    values = list(values)
    if not values:
        return 0.0
    return sum(values) / len(values)


def compute_metrics(
    run_name: str,
    source_type: str,
    source_name: str,
    rows: List[Dict[str, float]],
    jump_m: float,
    jump_yaw_deg: float,
) -> Dict[str, object]:
    times, translation_steps, yaw_steps = step_series(rows)
    if rows:
        first = rows[0]
        last = rows[-1]
        dx = last["x"] - first["x"]
        dy = last["y"] - first["y"]
        dz = last["z"] - first["z"]
        final_displacement = math.sqrt(dx * dx + dy * dy + dz * dz)
        final_yaw_delta = math.degrees(angle_diff(last["yaw"], first["yaw"]))
        first_t = rows[0].get("t", rows[0]["elapsed"])
        last_t = rows[-1].get("t", rows[-1]["elapsed"])
    else:
        final_displacement = 0.0
        final_yaw_delta = 0.0
        first_t = 0.0
        last_t = 0.0

    return {
        "run": run_name,
        "source_type": source_type,
        "source": source_name,
        "sample_count": len(rows),
        "first_t_sec": round(first_t, 6),
        "last_t_sec": round(last_t, 6),
        "path_length_m": round(sum(translation_steps), 6),
        "final_displacement_m": round(final_displacement, 6),
        "final_yaw_delta_deg": round(final_yaw_delta, 6),
        "max_translation_step_m": round(max(translation_steps) if translation_steps else 0.0, 6),
        "mean_translation_step_m": round(mean(translation_steps), 6),
        "translation_jump_count": sum(1 for value in translation_steps if value > jump_m),
        "max_yaw_step_deg": round(max(yaw_steps) if yaw_steps else 0.0, 6),
        "mean_yaw_step_deg": round(mean(yaw_steps), 6),
        "yaw_jump_count": sum(1 for value in yaw_steps if value > jump_yaw_deg),
    }


def write_summary_csv(path: Path, rows: List[Dict[str, object]]) -> None:
    if not rows:
        return
    fields = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def plot_time_series(ax, rows: List[Dict[str, float]], key: str, label: str, color: str, linestyle: str = "-"):
    if not rows:
        return
    ax.plot([row["t"] for row in rows], [row[key] for row in rows], label=label, color=color, linestyle=linestyle)


def plot_yaw_series(ax, rows: List[Dict[str, float]], label: str, color: str, linestyle: str = "-"):
    if not rows:
        return
    yaw_deg = [math.degrees(value) for value in unwrap_angles([row["yaw"] for row in rows])]
    ax.plot([row["t"] for row in rows], yaw_deg, label=label, color=color, linestyle=linestyle)


def plot_compare(args, data: Dict[str, Dict], out_path: Path) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required for plotting. Install it on the robot with: "
            "sudo apt install python3-matplotlib"
        ) from exc

    colors = {"2d": "tab:blue", "3d": "tab:orange"}
    fig, axes = plt.subplots(3, 2, figsize=(15, 12), constrained_layout=True)

    ax_xy = axes[0][0]
    for run_name, run in data.items():
        rows = run["tf_map_base"]
        if not rows:
            continue
        ax_xy.plot(
            [row["x"] for row in rows],
            [row["y"] for row in rows],
            label=run_name.upper(),
            color=colors[run_name],
        )
        ax_xy.scatter([rows[0]["x"]], [rows[0]["y"]], color=colors[run_name], marker="o", s=24)
        ax_xy.scatter([rows[-1]["x"]], [rows[-1]["y"]], color=colors[run_name], marker="x", s=36)
    ax_xy.set_title("map->base_link XY trajectory")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.axis("equal")
    ax_xy.grid(True)
    ax_xy.legend()

    ax_yaw = axes[0][1]
    for run_name, run in data.items():
        plot_yaw_series(ax_yaw, run["tf_map_base"], f"{run_name.upper()} yaw", colors[run_name])
    ax_yaw.axvline(0.0, color="0.5", linewidth=1.0, linestyle=":")
    ax_yaw.set_title("map->base_link yaw")
    ax_yaw.set_xlabel("time from drive start [s]")
    ax_yaw.set_ylabel("yaw [deg, unwrapped]")
    ax_yaw.grid(True)
    ax_yaw.legend()

    ax_xy_time = axes[1][0]
    for run_name, run in data.items():
        plot_time_series(ax_xy_time, run["tf_map_base"], "x", f"{run_name.upper()} x", colors[run_name], "-")
        plot_time_series(ax_xy_time, run["tf_map_base"], "y", f"{run_name.upper()} y", colors[run_name], "--")
    ax_xy_time.axvline(0.0, color="0.5", linewidth=1.0, linestyle=":")
    ax_xy_time.set_title("map->base_link x/y")
    ax_xy_time.set_xlabel("time from drive start [s]")
    ax_xy_time.set_ylabel("position [m]")
    ax_xy_time.grid(True)
    ax_xy_time.legend(ncol=2)

    ax_map_odom = axes[1][1]
    for run_name, run in data.items():
        rows = run["tf_map_odom"]
        if not rows:
            continue
        times = [row["t"] for row in rows]
        correction_norm = [math.sqrt(row["x"] ** 2 + row["y"] ** 2 + row["z"] ** 2) for row in rows]
        yaw_deg = [math.degrees(value) for value in unwrap_angles([row["yaw"] for row in rows])]
        ax_map_odom.plot(times, correction_norm, label=f"{run_name.upper()} |xyz| m", color=colors[run_name])
        ax_map_odom.plot(times, yaw_deg, label=f"{run_name.upper()} yaw deg", color=colors[run_name], linestyle="--")
    ax_map_odom.axvline(0.0, color="0.5", linewidth=1.0, linestyle=":")
    ax_map_odom.set_title("map->odom correction")
    ax_map_odom.set_xlabel("time from drive start [s]")
    ax_map_odom.set_ylabel("translation [m] / yaw [deg]")
    ax_map_odom.grid(True)
    ax_map_odom.legend(ncol=2)

    ax_trans_jump = axes[2][0]
    ax_yaw_jump = axes[2][1]
    for run_name, run in data.items():
        times, translation_steps, yaw_steps = step_series(run["tf_map_base"])
        ax_trans_jump.plot(times, translation_steps, label=run_name.upper(), color=colors[run_name])
        ax_yaw_jump.plot(times, yaw_steps, label=run_name.upper(), color=colors[run_name])
    ax_trans_jump.axhline(args.jump_m, color="red", linestyle=":", label=f"threshold {args.jump_m:.3f} m")
    ax_trans_jump.axvline(0.0, color="0.5", linewidth=1.0, linestyle=":")
    ax_trans_jump.set_title("map->base_link translation step")
    ax_trans_jump.set_xlabel("time from drive start [s]")
    ax_trans_jump.set_ylabel("step [m/sample]")
    ax_trans_jump.grid(True)
    ax_trans_jump.legend()

    ax_yaw_jump.axhline(args.jump_yaw_deg, color="red", linestyle=":", label=f"threshold {args.jump_yaw_deg:.1f} deg")
    ax_yaw_jump.axvline(0.0, color="0.5", linewidth=1.0, linestyle=":")
    ax_yaw_jump.set_title("map->base_link yaw step")
    ax_yaw_jump.set_xlabel("time from drive start [s]")
    ax_yaw_jump.set_ylabel("step [deg/sample]")
    ax_yaw_jump.grid(True)
    ax_yaw_jump.legend()

    fig.suptitle("2D AMCL vs 3D VGICP localization comparison", fontsize=14)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def collect_run_data(args, run_name: str, run_dir: Path) -> Dict:
    summary = read_json(run_dir / "summary.json")
    drive_start = float(summary.get("drive_start_elapsed_sec", 0.0))

    tf_map_base = normalize_time(read_tf_rows(run_dir, args.map_frame, args.base_frame), drive_start)
    tf_map_odom = normalize_time(read_tf_rows(run_dir, args.map_frame, args.odom_frame), drive_start)
    tf_odom_base = normalize_time(read_tf_rows(run_dir, args.odom_frame, args.base_frame), drive_start)
    raw_amcl = normalize_time(read_raw_topic_rows(run_dir, args.amcl_topic), drive_start)
    raw_localized = normalize_time(read_raw_topic_rows(run_dir, args.localized_topic), drive_start)
    raw_odom = normalize_time(read_raw_topic_rows(run_dir, args.odom_topic), drive_start)

    return {
        "run_dir": str(run_dir),
        "summary": summary,
        "tf_map_base": tf_map_base,
        "tf_map_odom": tf_map_odom,
        "tf_odom_base": tf_odom_base,
        "raw_amcl": raw_amcl,
        "raw_localized": raw_localized,
        "raw_odom": raw_odom,
    }


def build_metric_rows(args, data: Dict[str, Dict]) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    for run_name, run in data.items():
        sources = [
            ("tf", f"{args.map_frame}->{args.base_frame}", run["tf_map_base"]),
            ("tf", f"{args.map_frame}->{args.odom_frame}", run["tf_map_odom"]),
            ("tf", f"{args.odom_frame}->{args.base_frame}", run["tf_odom_base"]),
            ("raw", args.amcl_topic, run["raw_amcl"]),
            ("raw", args.localized_topic, run["raw_localized"]),
            ("raw", args.odom_topic, run["raw_odom"]),
        ]
        for source_type, source_name, source_rows in sources:
            rows.append(
                compute_metrics(
                    run_name,
                    source_type,
                    source_name,
                    source_rows,
                    args.jump_m,
                    args.jump_yaw_deg,
                )
            )
    return rows


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Compare 2D and 3D localization trial outputs.")
    parser.add_argument("--run-2d", required=True, type=Path, help="Directory generated by record_trial.py for 2D.")
    parser.add_argument("--run-3d", required=True, type=Path, help="Directory generated by record_trial.py for 3D.")
    parser.add_argument("--out", default="~/loc_compare/localization_compare.png", type=Path, help="Output PNG path.")
    parser.add_argument("--jump-m", default=0.15, type=float, help="Translation step jump threshold.")
    parser.add_argument("--jump-yaw-deg", default=5.0, type=float, help="Yaw step jump threshold.")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--amcl-topic", default="/amcl_pose")
    parser.add_argument("--localized-topic", default="/localizer/localized_result")
    parser.add_argument("--odom-topic", default="/odom")
    return parser


def main(argv=None) -> int:
    args = build_arg_parser().parse_args(argv)
    args.run_2d = args.run_2d.expanduser()
    args.run_3d = args.run_3d.expanduser()
    args.out = args.out.expanduser()

    if not args.run_2d.is_dir():
        raise SystemExit(f"2D run directory does not exist: {args.run_2d}")
    if not args.run_3d.is_dir():
        raise SystemExit(f"3D run directory does not exist: {args.run_3d}")

    data = {
        "2d": collect_run_data(args, "2d", args.run_2d),
        "3d": collect_run_data(args, "3d", args.run_3d),
    }
    metric_rows = build_metric_rows(args, data)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    summary_csv = args.out.with_name("compare_summary.csv")
    summary_json = args.out.with_name("compare_summary.json")
    write_summary_csv(summary_csv, metric_rows)
    with summary_json.open("w", encoding="utf-8") as handle:
        json.dump(
            {
                "run_2d": str(args.run_2d),
                "run_3d": str(args.run_3d),
                "jump_m": args.jump_m,
                "jump_yaw_deg": args.jump_yaw_deg,
                "metrics": metric_rows,
            },
            handle,
            indent=2,
            ensure_ascii=False,
        )

    plot_compare(args, data, args.out)
    print(f"plot: {args.out}")
    print(f"summary csv: {summary_csv}")
    print(f"summary json: {summary_json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
