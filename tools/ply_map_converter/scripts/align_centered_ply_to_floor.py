#!/usr/bin/env python3
"""Align a head-camera-centered PLY cloud to a floor-based map frame.

The output remains a binary little-endian PLY with the same vertex properties,
but x/y/z are transformed so x/y lie on the fitted floor plane and z is height
above that plane. Normals are transformed when nx/ny/nz fields are present.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


PLY_NUMPY_TYPES = {
    "char": "i1",
    "int8": "i1",
    "uchar": "u1",
    "uint8": "u1",
    "short": "<i2",
    "int16": "<i2",
    "ushort": "<u2",
    "uint16": "<u2",
    "int": "<i4",
    "int32": "<i4",
    "uint": "<u4",
    "uint32": "<u4",
    "float": "<f4",
    "float32": "<f4",
    "double": "<f8",
    "float64": "<f8",
}


@dataclass(frozen=True)
class PlyProperty:
    type_name: str
    name: str


@dataclass(frozen=True)
class PlyInfo:
    path: Path
    header_offset: int
    vertex_count: int
    properties: list[PlyProperty]
    dtype: np.dtype


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fit a dominant floor plane in a centered PLY point cloud and "
            "write an aligned PLY whose z=0 is the floor."
        )
    )
    parser.add_argument("--input", required=True, type=Path, help="Input centered PLY")
    parser.add_argument("--output", required=True, type=Path, help="Output aligned PLY")
    parser.add_argument(
        "--metadata-output",
        type=Path,
        default=None,
        help="Optional transform metadata JSON; defaults to <output>.json",
    )
    parser.add_argument(
        "--no-metadata",
        action="store_true",
        help="Do not write transform metadata JSON",
    )
    parser.add_argument(
        "--sample-size",
        type=int,
        default=250_000,
        help="Random point sample used for RANSAC, default: 250000",
    )
    parser.add_argument(
        "--ransac-iterations",
        type=int,
        default=600,
        help="RANSAC plane iterations, default: 600",
    )
    parser.add_argument(
        "--plane-threshold",
        type=float,
        default=0.035,
        help="Plane inlier threshold in meters, default: 0.035",
    )
    parser.add_argument(
        "--refit-sample-size",
        type=int,
        default=400_000,
        help="Maximum full-cloud inliers sampled for PCA plane refit, default: 400000",
    )
    parser.add_argument(
        "--min-camera-height",
        type=float,
        default=0.5,
        help="Reject candidate planes closer than this to the centered camera, default: 0.5",
    )
    parser.add_argument(
        "--max-camera-height",
        type=float,
        default=3.5,
        help="Reject candidate planes farther than this from the centered camera, default: 3.5",
    )
    parser.add_argument(
        "--forward-axis",
        choices=["x", "y", "z", "-x", "-y", "-z"],
        default="z",
        help="Raw centered axis projected onto the floor as map +X, default: z",
    )
    parser.add_argument(
        "--plane-normal",
        default=None,
        help="Manual raw plane normal as 'nx,ny,nz'; skips RANSAC when used with --plane-d",
    )
    parser.add_argument(
        "--plane-d",
        type=float,
        default=None,
        help="Manual raw plane offset d for normal.dot(point)+d=0",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=7,
        help="Random seed for deterministic fitting, default: 7",
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=1_000_000,
        help="Vertices per output write chunk, default: 1000000",
    )
    return parser.parse_args()


def parse_ply(path: Path) -> PlyInfo:
    with path.open("rb") as stream:
        first = stream.readline()
        if first.rstrip() != b"ply":
            raise ValueError(f"input is not a PLY file: {path}")

        fmt = None
        vertex_count = None
        properties: list[PlyProperty] = []
        current_element = None
        extra_elements: list[str] = []

        while True:
            raw = stream.readline()
            if not raw:
                raise ValueError("PLY header is missing end_header")
            line = raw.decode("ascii").strip()
            if not line:
                continue
            fields = line.split()
            token = fields[0]
            if token == "format":
                fmt = fields[1]
            elif token == "element":
                current_element = fields[1]
                count = int(fields[2])
                if current_element == "vertex":
                    vertex_count = count
                elif count > 0:
                    extra_elements.append(current_element)
            elif token == "property":
                if current_element != "vertex":
                    continue
                if len(fields) >= 2 and fields[1] == "list":
                    raise ValueError("list properties are not supported on vertices")
                if len(fields) != 3:
                    raise ValueError(f"invalid PLY property line: {line}")
                type_name, name = fields[1], fields[2]
                if type_name not in PLY_NUMPY_TYPES:
                    raise ValueError(f"unsupported PLY property type: {type_name}")
                properties.append(PlyProperty(type_name=type_name, name=name))
            elif token == "end_header":
                header_offset = stream.tell()
                break

    if fmt != "binary_little_endian":
        raise ValueError(f"unsupported PLY format {fmt!r}; expected binary_little_endian")
    if extra_elements:
        joined = ", ".join(extra_elements)
        raise ValueError(f"PLY contains non-vertex elements that cannot be preserved: {joined}")
    if vertex_count is None or vertex_count <= 0:
        raise ValueError("PLY has no vertex element")
    names = [prop.name for prop in properties]
    for required in ("x", "y", "z"):
        if required not in names:
            raise ValueError(f"PLY vertex is missing required property: {required}")

    dtype = np.dtype([(prop.name, PLY_NUMPY_TYPES[prop.type_name]) for prop in properties])
    return PlyInfo(
        path=path,
        header_offset=header_offset,
        vertex_count=vertex_count,
        properties=properties,
        dtype=dtype,
    )


def load_vertices(info: PlyInfo) -> np.memmap:
    return np.memmap(
        info.path,
        dtype=info.dtype,
        mode="r",
        offset=info.header_offset,
        shape=(info.vertex_count,),
    )


def xyz_from_records(records: np.ndarray) -> np.ndarray:
    return np.column_stack((records["x"], records["y"], records["z"])).astype(np.float64)


def bounds(points: np.ndarray) -> dict[str, list[float]]:
    return {
        "min": points.min(axis=0).tolist(),
        "max": points.max(axis=0).tolist(),
        "p01": np.percentile(points, 1, axis=0).tolist(),
        "p50": np.percentile(points, 50, axis=0).tolist(),
        "p99": np.percentile(points, 99, axis=0).tolist(),
    }


def parse_vector(value: str) -> np.ndarray:
    parts = [float(part.strip()) for part in value.split(",")]
    if len(parts) != 3:
        raise ValueError(f"expected three comma-separated values, got: {value}")
    vector = np.asarray(parts, dtype=np.float64)
    norm = np.linalg.norm(vector)
    if norm <= 1.0e-9:
        raise ValueError("vector must be non-zero")
    return vector / norm


def axis_vector(axis: str) -> np.ndarray:
    sign = -1.0 if axis.startswith("-") else 1.0
    name = axis[-1]
    if name == "x":
        return np.array([sign, 0.0, 0.0], dtype=np.float64)
    if name == "y":
        return np.array([0.0, sign, 0.0], dtype=np.float64)
    return np.array([0.0, 0.0, sign], dtype=np.float64)


def orient_plane_for_camera_above(normal: np.ndarray, offset: float) -> tuple[np.ndarray, float]:
    normal = normal / np.linalg.norm(normal)
    if -offset < 0.0:
        normal = -normal
        offset = -offset
    return normal, offset


def fit_plane_pca(points: np.ndarray) -> tuple[np.ndarray, float]:
    center = points.mean(axis=0)
    _, _, vh = np.linalg.svd(points - center, full_matrices=False)
    normal = vh[-1]
    offset = -float(normal.dot(center))
    return orient_plane_for_camera_above(normal, offset)


def ransac_plane(
    sample: np.ndarray,
    rng: np.random.Generator,
    iterations: int,
    threshold: float,
    min_height: float,
    max_height: float,
) -> tuple[np.ndarray, float, int]:
    best_count = -1
    best_normal = None
    best_offset = None
    point_count = sample.shape[0]

    for _ in range(iterations):
        ids = rng.choice(point_count, size=3, replace=False)
        p0, p1, p2 = sample[ids]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm <= 1.0e-9:
            continue
        normal /= norm
        offset = -float(normal.dot(p0))
        height = abs(offset)
        if height < min_height or height > max_height:
            continue

        distance = np.abs(sample @ normal + offset)
        count = int(np.count_nonzero(distance <= threshold))
        if count > best_count:
            best_count = count
            best_normal = normal.copy()
            best_offset = offset

    if best_normal is None or best_offset is None:
        raise RuntimeError("failed to find a floor plane candidate with the current thresholds")

    normal, offset = orient_plane_for_camera_above(best_normal, float(best_offset))
    return normal, offset, best_count


def build_map_axes(plane_normal: np.ndarray, plane_offset: float, forward_axis: str) -> dict[str, np.ndarray | float]:
    z_axis = -plane_normal
    z_axis /= np.linalg.norm(z_axis)

    forward = axis_vector(forward_axis)
    x_axis = forward - float(forward.dot(z_axis)) * z_axis
    x_norm = np.linalg.norm(x_axis)
    if x_norm <= 1.0e-6:
        candidates = [axis_vector("x"), axis_vector("y"), axis_vector("z")]
        forward = min(candidates, key=lambda candidate: abs(float(candidate.dot(z_axis))))
        x_axis = forward - float(forward.dot(z_axis)) * z_axis
        x_norm = np.linalg.norm(x_axis)
    x_axis /= x_norm

    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    camera_height = -plane_offset
    return {
        "x_axis": x_axis,
        "y_axis": y_axis,
        "z_axis": z_axis,
        "camera_height": camera_height,
    }


def transform_xyz(points: np.ndarray, axes: dict[str, np.ndarray | float]) -> np.ndarray:
    transformed = np.empty_like(points, dtype=np.float64)
    transformed[:, 0] = points @ axes["x_axis"]
    transformed[:, 1] = points @ axes["y_axis"]
    transformed[:, 2] = points @ axes["z_axis"] + float(axes["camera_height"])
    return transformed


def transform_normals(records: np.ndarray, axes: dict[str, np.ndarray | float]) -> None:
    if not all(name in records.dtype.names for name in ("nx", "ny", "nz")):
        return
    normals = np.column_stack((records["nx"], records["ny"], records["nz"])).astype(np.float64)
    transformed = np.empty_like(normals)
    transformed[:, 0] = normals @ axes["x_axis"]
    transformed[:, 1] = normals @ axes["y_axis"]
    transformed[:, 2] = normals @ axes["z_axis"]
    records["nx"] = transformed[:, 0].astype(records.dtype["nx"])
    records["ny"] = transformed[:, 1].astype(records.dtype["ny"])
    records["nz"] = transformed[:, 2].astype(records.dtype["nz"])


def make_output_header(info: PlyInfo, metadata: dict[str, object]) -> bytes:
    comments = [
        "comment aligned_by align_centered_ply_to_floor.py",
        f"comment source {metadata['input']}",
        (
            "comment floor_plane normal "
            + ",".join(f"{value:.10g}" for value in metadata["plane_normal"])
            + f" d {metadata['plane_d']:.10g}"
        ),
        f"comment camera_height {metadata['camera_height']:.10g}",
    ]
    lines = ["ply", "format binary_little_endian 1.0", *comments]
    lines.append(f"element vertex {info.vertex_count}")
    for prop in info.properties:
        lines.append(f"property {prop.type_name} {prop.name}")
    lines.append("end_header")
    return ("\n".join(lines) + "\n").encode("ascii")


def iter_chunks(count: int, chunk_size: int) -> Iterable[tuple[int, int]]:
    for start in range(0, count, chunk_size):
        yield start, min(start + chunk_size, count)


def write_aligned_ply(
    info: PlyInfo,
    vertices: np.memmap,
    output: Path,
    axes: dict[str, np.ndarray | float],
    metadata: dict[str, object],
    chunk_size: int,
) -> dict[str, object]:
    output.parent.mkdir(parents=True, exist_ok=True)
    header = make_output_header(info, metadata)
    mins = np.full(3, np.inf)
    maxs = np.full(3, -np.inf)
    free_points = 0
    occupied_points = 0
    occupied_2m_points = 0

    with output.open("wb") as stream:
        stream.write(header)
        for start, stop in iter_chunks(info.vertex_count, chunk_size):
            chunk = np.array(vertices[start:stop], copy=True)
            xyz = xyz_from_records(chunk)
            aligned = transform_xyz(xyz, axes)
            chunk["x"] = aligned[:, 0].astype(chunk.dtype["x"])
            chunk["y"] = aligned[:, 1].astype(chunk.dtype["y"])
            chunk["z"] = aligned[:, 2].astype(chunk.dtype["z"])
            transform_normals(chunk, axes)
            stream.write(chunk.tobytes())

            mins = np.minimum(mins, aligned.min(axis=0))
            maxs = np.maximum(maxs, aligned.max(axis=0))
            z = aligned[:, 2]
            free_points += int(np.count_nonzero((z >= -0.05) & (z <= 0.08)))
            occupied_points += int(np.count_nonzero((z >= 0.15) & (z <= 1.70)))
            occupied_2m_points += int(np.count_nonzero((z >= 0.15) & (z <= 2.0)))

    return {
        "min": mins.tolist(),
        "max": maxs.tolist(),
        "free_band_points": free_points,
        "occupied_band_points": occupied_points,
        "occupied_2m_band_points": occupied_2m_points,
    }


def main() -> int:
    args = parse_args()
    if args.sample_size <= 0:
        raise ValueError("--sample-size must be positive")
    if args.ransac_iterations <= 0:
        raise ValueError("--ransac-iterations must be positive")
    if args.plane_threshold <= 0.0:
        raise ValueError("--plane-threshold must be positive")
    if args.refit_sample_size <= 0:
        raise ValueError("--refit-sample-size must be positive")
    if args.chunk_size <= 0:
        raise ValueError("--chunk-size must be positive")
    if (args.plane_normal is None) != (args.plane_d is None):
        raise ValueError("--plane-normal and --plane-d must be provided together")

    info = parse_ply(args.input)
    vertices = load_vertices(info)
    xyz = xyz_from_records(vertices)
    finite = np.isfinite(xyz).all(axis=1)
    if not np.all(finite):
        xyz = xyz[finite]
    if xyz.shape[0] < 3:
        raise RuntimeError("not enough finite points to fit a plane")

    rng = np.random.default_rng(args.seed)
    if args.plane_normal is not None:
        normal, offset = orient_plane_for_camera_above(parse_vector(args.plane_normal), args.plane_d)
        initial_inliers = int(np.count_nonzero(np.abs(xyz @ normal + offset) <= args.plane_threshold))
    else:
        sample_count = min(args.sample_size, xyz.shape[0])
        sample_ids = np.sort(rng.choice(xyz.shape[0], size=sample_count, replace=False))
        sample = xyz[sample_ids]
        normal, offset, initial_inliers = ransac_plane(
            sample,
            rng,
            args.ransac_iterations,
            args.plane_threshold,
            args.min_camera_height,
            args.max_camera_height,
        )

        full_inlier_mask = np.abs(xyz @ normal + offset) <= args.plane_threshold
        full_inliers = xyz[full_inlier_mask]
        if full_inliers.shape[0] > args.refit_sample_size:
            refit_ids = rng.choice(full_inliers.shape[0], size=args.refit_sample_size, replace=False)
            full_inliers = full_inliers[refit_ids]
        normal, offset = fit_plane_pca(full_inliers)

    axes = build_map_axes(normal, offset, args.forward_axis)
    output = args.output
    metadata_output = args.metadata_output
    if metadata_output is None:
        metadata_output = output.with_suffix(output.suffix + ".json")

    sample_for_percentiles = xyz
    if sample_for_percentiles.shape[0] > 500_000:
        ids = np.sort(rng.choice(sample_for_percentiles.shape[0], size=500_000, replace=False))
        sample_for_percentiles = sample_for_percentiles[ids]
    aligned_sample = transform_xyz(sample_for_percentiles, axes)

    metadata: dict[str, object] = {
        "input": str(args.input),
        "output": str(output),
        "vertex_count": int(info.vertex_count),
        "plane_normal": normal.tolist(),
        "plane_d": float(offset),
        "camera_height": float(axes["camera_height"]),
        "map_axes": {
            "x": axes["x_axis"].tolist(),
            "y": axes["y_axis"].tolist(),
            "z": axes["z_axis"].tolist(),
        },
        "forward_axis": args.forward_axis,
        "ransac_sample_inliers": int(initial_inliers),
        "raw_bounds": bounds(sample_for_percentiles),
        "aligned_sample_bounds": bounds(aligned_sample),
        "plane_threshold": args.plane_threshold,
    }

    aligned_stats = write_aligned_ply(
        info=info,
        vertices=vertices,
        output=output,
        axes=axes,
        metadata=metadata,
        chunk_size=args.chunk_size,
    )
    metadata["aligned_bounds"] = {
        "min": aligned_stats["min"],
        "max": aligned_stats["max"],
    }
    metadata["height_band_counts"] = {
        "free_-0.05_0.08": aligned_stats["free_band_points"],
        "occupied_0.15_1.70": aligned_stats["occupied_band_points"],
        "occupied_0.15_2.00": aligned_stats["occupied_2m_band_points"],
    }

    if not args.no_metadata:
        metadata_output.parent.mkdir(parents=True, exist_ok=True)
        metadata_output.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    print(f"Input: {args.input}")
    print(f"Output: {output}")
    print(f"Vertices: {info.vertex_count}")
    print(
        "Floor plane: normal=[{}] d={:.6f} camera_height={:.3f}m".format(
            ", ".join(f"{value:.6f}" for value in normal),
            float(offset),
            float(axes["camera_height"]),
        )
    )
    print(
        "Aligned bounds: x=[{:.3f}, {:.3f}] y=[{:.3f}, {:.3f}] z=[{:.3f}, {:.3f}]".format(
            aligned_stats["min"][0],
            aligned_stats["max"][0],
            aligned_stats["min"][1],
            aligned_stats["max"][1],
            aligned_stats["min"][2],
            aligned_stats["max"][2],
        )
    )
    print(
        "Height bands: free(-0.05..0.08)={} occupied(0.15..1.70)={} occupied(0.15..2.00)={}".format(
            aligned_stats["free_band_points"],
            aligned_stats["occupied_band_points"],
            aligned_stats["occupied_2m_band_points"],
        )
    )
    if not args.no_metadata:
        print(f"Metadata: {metadata_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
