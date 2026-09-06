#!/usr/bin/env python3
import argparse
import os
from typing import List, Sequence, Tuple

import numpy as np
import trimesh
import yaml
from PIL import Image
from skimage.draw import line as draw_line
from skimage.draw import polygon as draw_polygon
from yourdfpy import URDF


FREE = np.uint8(254)
OCCUPIED = np.uint8(0)


def make_filename_handler(package_dir: str, urdf_dir: str):
    """Resolve package://, file://, absolute and relative mesh paths."""
    package_dir = os.path.abspath(package_dir)
    urdf_dir = os.path.abspath(urdf_dir)
    local_package_name = os.path.basename(package_dir.rstrip(os.sep))

    def handler(fname: str) -> str:
        if not fname:
            return fname

        if fname.startswith("file://"):
            return os.path.normpath(fname[7:])

        if fname.startswith("package://"):
            package_uri = fname[len("package://"):]
            parts = package_uri.split("/", 1)
            if len(parts) != 2:
                return fname

            package_name, rel_path = parts
            candidates = []

            if package_name in ("iai_apartment", local_package_name):
                candidates.append(os.path.join(package_dir, rel_path))

            for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
                if prefix:
                    candidates.append(
                        os.path.join(prefix, "share", package_name, rel_path)
                    )

            for candidate in candidates:
                if os.path.exists(candidate):
                    return os.path.normpath(candidate)

            return os.path.normpath(candidates[0]) if candidates else fname

        if os.path.isabs(fname):
            return os.path.normpath(fname)

        for base in (urdf_dir, package_dir):
            candidate = os.path.join(base, fname)
            if os.path.exists(candidate):
                return os.path.normpath(candidate)

        return os.path.normpath(os.path.join(urdf_dir, fname))

    return handler


def load_world_collision_meshes(
    urdf_path: str,
    package_dir: str,
    exclude_patterns: Sequence[str],
    root_xyz: Sequence[float],
    root_rpy: Sequence[float],
) -> List[Tuple[str, trimesh.Trimesh]]:
    """
    Let yourdfpy build the collision scene, then bake every collision geometry
    into the URDF root frame. This preserves URDF mesh scale and transforms.
    """
    urdf_path = os.path.abspath(urdf_path)
    resolver = make_filename_handler(package_dir, os.path.dirname(urdf_path))

    robot = URDF.load(
        urdf_path,
        filename_handler=resolver,
        build_scene_graph=False,
        build_collision_scene_graph=True,
        load_meshes=False,
        load_collision_meshes=True,
        force_collision_mesh=False,
    )

    scene = robot.collision_scene
    if scene is None or len(scene.geometry) == 0:
        raise RuntimeError(
            "No collision scene was loaded. Check <collision> elements and mesh paths."
        )

    root_transform = trimesh.transformations.euler_matrix(
        root_rpy[0], root_rpy[1], root_rpy[2], axes="sxyz"
    )
    root_transform[:3, 3] = np.asarray(root_xyz, dtype=float)

    meshes = []
    for node_name in scene.graph.nodes_geometry:
        transform, geom_name = scene.graph.get(frame_to=node_name)
        label = f"{node_name} {geom_name}"
        label_lower = label.lower()

        if any(p.lower() in label_lower for p in exclude_patterns):
            print(f"Skipping: {label}")
            continue

        geom = scene.geometry.get(geom_name)
        if not isinstance(geom, trimesh.Trimesh):
            print(f"Warning: non-mesh collision geometry skipped: {label}")
            continue

        mesh = geom.copy()
        mesh.apply_transform(root_transform @ transform)
        mesh.remove_unreferenced_vertices()

        if len(mesh.vertices) and len(mesh.faces):
            meshes.append((label, mesh))

    if not meshes:
        raise RuntimeError("No usable collision meshes remain after filtering.")

    return meshes


def band_xy_bounds(meshes, z_min: float, z_max: float) -> np.ndarray:
    """XY bounds of triangles that intersect the requested Z band."""
    xy = []

    for mesh in meshes:
        tri = mesh.triangles
        tri_min_z = tri[:, :, 2].min(axis=1)
        tri_max_z = tri[:, :, 2].max(axis=1)
        keep = (tri_max_z >= z_min) & (tri_min_z <= z_max)
        if np.any(keep):
            xy.append(tri[keep, :, :2].reshape(-1, 2))

    if not xy:
        raise ValueError(
            f"No collision geometry intersects z=[{z_min:.3f}, {z_max:.3f}] m"
        )

    xy = np.vstack(xy)
    return np.array(
        [[xy[:, 0].min(), xy[:, 1].min()],
         [xy[:, 0].max(), xy[:, 1].max()]],
        dtype=float,
    )


def to_world_xy(points_2d: np.ndarray, to_3d: np.ndarray) -> np.ndarray:
    points_2d = np.asarray(points_2d, dtype=float)
    p = np.column_stack(
        [points_2d[:, 0], points_2d[:, 1], np.zeros(len(points_2d)), np.ones(len(points_2d))]
    )
    return (to_3d @ p.T).T[:, :2]


def xy_to_grid(xy, min_x, min_y, resolution):
    cols = (xy[:, 0] - min_x) / resolution
    rows = (xy[:, 1] - min_y) / resolution
    return rows, cols


def fill_polygon_world(mask, xy, min_x, min_y, resolution, value=True):
    if len(xy) < 3:
        return
    rows, cols = xy_to_grid(xy, min_x, min_y, resolution)
    rr, cc = draw_polygon(rows, cols, shape=mask.shape)
    mask[rr, cc] = value


def draw_polyline_world(mask, xy, min_x, min_y, resolution):
    if len(xy) < 2:
        return

    rows, cols = xy_to_grid(xy, min_x, min_y, resolution)
    rows = np.rint(rows).astype(int)
    cols = np.rint(cols).astype(int)

    for i in range(len(xy) - 1):
        rr, cc = draw_line(rows[i], cols[i], rows[i + 1], cols[i + 1])
        valid = (
            (rr >= 0) & (rr < mask.shape[0]) &
            (cc >= 0) & (cc < mask.shape[1])
        )
        mask[rr[valid], cc[valid]] = True


def rasterize_path2d(path, shape, min_x, min_y, resolution):
    """Rasterize one horizontal cross-section while preserving polygon holes."""
    result = np.zeros(shape, dtype=bool)
    to_3d = path.metadata.get("to_3D")
    if to_3d is None:
        raise RuntimeError("section_multiplane returned a path without to_3D metadata")

    # Fill actual solid cross-sectional areas, preserving holes.
    try:
        for poly in path.polygons_full:
            if poly is None or poly.is_empty:
                continue

            poly_mask = np.zeros(shape, dtype=bool)
            exterior = to_world_xy(np.asarray(poly.exterior.coords), to_3d)
            fill_polygon_world(
                poly_mask, exterior, min_x, min_y, resolution, True
            )

            for hole in poly.interiors:
                hole_xy = to_world_xy(np.asarray(hole.coords), to_3d)
                fill_polygon_world(
                    poly_mask, hole_xy, min_x, min_y, resolution, False
                )

            result |= poly_mask
    except Exception as exc:
        print(f"Warning: polygon fill failed for one slice: {exc}")

    # Also draw section curves, so open/non-watertight collision meshes remain visible.
    for curve_2d in path.discrete:
        if len(curve_2d) >= 2:
            curve_xy = to_world_xy(np.asarray(curve_2d), to_3d)
            draw_polyline_world(
                result, curve_xy, min_x, min_y, resolution
            )

    return result


def slice_heights_for_mesh(mesh, z_min, z_max, z_step):
    low = max(z_min, float(mesh.bounds[0, 2]))
    high = min(z_max, float(mesh.bounds[1, 2]))
    if high < low:
        return np.empty(0, dtype=float)

    # Regular slices plus a guaranteed midpoint prevent thin objects from
    # falling completely between two regular slice planes.
    first = np.ceil((low - z_min) / z_step) * z_step + z_min
    heights = list(np.arange(first, high + 0.5 * z_step, z_step))
    heights.append(0.5 * (low + high))
    return np.unique(np.round(heights, 6))


def urdf_to_occupancy_grid(
    urdf_path: str,
    package_dir: str,
    output_prefix: str = "apartment_map",
    resolution: float = 0.05,
    z_min: float = 0.15,
    z_max: float = 2.00,
    z_step: float = 0.025,
    padding: float = 0.50,
    exclude_patterns: Sequence[str] = ("floor", "ground", "ceiling"),
    root_xyz: Sequence[float] = (0.0, 0.0, 0.0),
    root_rpy: Sequence[float] = (0.0, 0.0, 0.0),
):
    if resolution <= 0 or z_step <= 0:
        raise ValueError("resolution and z_step must be > 0")
    if z_max <= z_min:
        raise ValueError("z_max must be greater than z_min")

    urdf_path = os.path.abspath(urdf_path)
    package_dir = os.path.abspath(package_dir)
    output_prefix = os.path.abspath(output_prefix)
    os.makedirs(os.path.dirname(output_prefix), exist_ok=True)

    print(f"Loading URDF: {urdf_path}")
    named_meshes = load_world_collision_meshes(
        urdf_path, package_dir, exclude_patterns, root_xyz, root_rpy
    )

    active = [
        (name, mesh) for name, mesh in named_meshes
        if mesh.bounds[1, 2] >= z_min and mesh.bounds[0, 2] <= z_max
    ]
    if not active:
        raise ValueError("No collision geometry overlaps the requested Z band")

    meshes = [mesh for _, mesh in active]
    bounds = band_xy_bounds(meshes, z_min, z_max)

    min_x = np.floor((bounds[0, 0] - padding) / resolution) * resolution
    min_y = np.floor((bounds[0, 1] - padding) / resolution) * resolution
    max_x = np.ceil((bounds[1, 0] + padding) / resolution) * resolution
    max_y = np.ceil((bounds[1, 1] + padding) / resolution) * resolution

    width = int(round((max_x - min_x) / resolution))
    height = int(round((max_y - min_y) / resolution))
    occupied = np.zeros((height, width), dtype=bool)

    print(f"Collision meshes in band: {len(active)}")
    print(f"Map: {width} x {height} cells @ {resolution:.3f} m/cell")
    print(f"Origin: [{min_x:.3f}, {min_y:.3f}, 0.0]")

    for index, (name, mesh) in enumerate(active, start=1):
        heights = slice_heights_for_mesh(mesh, z_min, z_max, z_step)
        if len(heights) == 0:
            continue

        sections = mesh.section_multiplane(
            plane_origin=[0.0, 0.0, 0.0],
            plane_normal=[0.0, 0.0, 1.0],
            heights=heights,
        )

        for path in sections:
            if path is not None:
                occupied |= rasterize_path2d(
                    path, occupied.shape, min_x, min_y, resolution
                )

        print(f"[{index:03d}/{len(active):03d}] {name}: {len(heights)} slices")

    if not occupied.any():
        raise RuntimeError(
            "Generated map contains no occupied cells. Check collision geometry and Z limits."
        )

    # Internal rows increase with +Y. PGM rows increase downward, so flip once.
    grid = np.full((height, width), FREE, dtype=np.uint8)
    grid[occupied] = OCCUPIED
    grid_ros = np.flipud(grid)

    pgm_path = output_prefix + ".pgm"
    yaml_path = output_prefix + ".yaml"
    Image.fromarray(grid_ros).save(pgm_path)

    metadata = {
        "image": os.path.basename(pgm_path),
        "mode": "trinary",
        "resolution": float(resolution),
        "origin": [float(min_x), float(min_y), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(metadata, f, sort_keys=False)

    print(f"Generated: {pgm_path}")
    print(f"Generated: {yaml_path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate a Nav2 occupancy map from URDF collision geometry"
    )
    parser.add_argument("urdf", help="Path to apartment.urdf")
    parser.add_argument("--package-dir", default=None)
    parser.add_argument("--output", default="apartment_map")
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--z-min", type=float, default=0.15)
    parser.add_argument("--z-max", type=float, default=2.00)
    parser.add_argument("--z-step", type=float, default=0.025)
    parser.add_argument("--padding", type=float, default=0.50)
    parser.add_argument(
        "--xyz", nargs=3, type=float, default=[0.0, 0.0, 0.0],
        metavar=("X", "Y", "Z"),
        help="Pose of the URDF root in the map/world frame [m]",
    )
    parser.add_argument(
        "--rpy", nargs=3, type=float, default=[0.0, 0.0, 0.0],
        metavar=("R", "P", "Y"),
        help="Pose of the URDF root in the map/world frame [rad]",
    )
    parser.add_argument(
        "--exclude",
        nargs="*",
        default=["floor", "ground", "ceiling"],
        help="Collision node/geometry name substrings to skip",
    )
    return parser.parse_args()


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))

    urdf_to_occupancy_grid(
        urdf_path=os.path.join(script_dir, "apartment.urdf"),
        package_dir=script_dir,
        output_prefix=os.path.join(script_dir, "apartment_map"),
        resolution=0.05,
        z_min=0.15,
        z_max=2.00,
        padding=0.5,
    )
