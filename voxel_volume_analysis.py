r"""
Estimate missing brick volume with a shared solid voxel grid.

This script is intended to run after align_icp.py. The healthy reference mesh
and aligned damaged mesh are placed on the same voxel grid. Each voxel center
is tested against the closed mesh surfaces, and the healthy-only voxels are
saved as the estimated missing region.

The OBJ files must already use the same physical units. Run
scale_brick_meshes.py before ICP, then use the scaled healthy mesh and the
aligned damaged output from align_icp.py.

Inputs
------
healthy:
    Healthy reference brick mesh.

damaged:
    Damaged brick mesh after ICP alignment.

Outputs
-------
voxel_volume_results.csv:
    Voxel counts, volumes, overlap measurements, and mesh checks.

missing_volume_voxels.ply:
    Centers of voxels occupied by the healthy brick but not the damaged brick.

damaged_only_voxels.ply:
    Centers of voxels occupied by the damaged brick but not the healthy brick.

voxel_overlay_colored.ply:
    Colored volume comparison. Gray is shared volume, blue is estimated
    missing volume, and red is damaged-only volume.

Example
-------
python .\icp\voxel_volume_analysis.py \
    --healthy .\meshes\scaled\healthy_brick.obj \
    --damaged .\icp\outputs\scaled\damaged_brick_1\aligned_source.obj \
    --out_dir .\icp\outputs\scaled\damaged_brick_1\voxel_volume \
    --voxel_size 0.005 \
    --units m \
    --visualize
"""

from pathlib import Path
import argparse
import csv

import numpy as np
import open3d as o3d


DEFAULT_VOXEL_SIZE = 0.005
DEFAULT_RAY_SAMPLES = 3
MAX_GRID_CELLS = 50000000
LARGE_EXTRA_FRACTION = 0.10
LOW_CONTAINMENT_FRACTION = 0.85


def read_mesh(file_path):
    file_path = Path(file_path)
    mesh = o3d.io.read_triangle_mesh(str(file_path))

    if mesh.is_empty():
        raise RuntimeError(f"Could not load mesh, or mesh is empty: {file_path}")

    mesh.remove_duplicated_vertices()
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    return mesh


def get_shared_grid(healthy_mesh, damaged_mesh, voxel_size):
    healthy_min = healthy_mesh.get_min_bound()
    healthy_max = healthy_mesh.get_max_bound()
    damaged_min = damaged_mesh.get_min_bound()
    damaged_max = damaged_mesh.get_max_bound()

    padding = voxel_size * 2
    grid_min = np.minimum(healthy_min, damaged_min) - padding
    grid_max = np.maximum(healthy_max, damaged_max) + padding
    grid_shape = np.ceil((grid_max - grid_min) / voxel_size).astype(int)

    grid_cells = int(np.prod(grid_shape, dtype=np.int64))
    if grid_cells > MAX_GRID_CELLS:
        raise RuntimeError(
            f"Voxel grid would contain {grid_cells:,} cells. "
            "Use a larger voxel size."
        )

    return grid_min, grid_shape


def mesh_surface_grid(mesh, grid_min, grid_shape, voxel_size):
    voxel_grid = o3d.geometry.VoxelGrid.create_from_triangle_mesh(
        mesh,
        voxel_size=voxel_size
    )

    surface = np.zeros(tuple(grid_shape), dtype=bool)
    voxel_origin = np.asarray(voxel_grid.origin)

    for voxel in voxel_grid.get_voxels():
        center = voxel_origin + (np.asarray(voxel.grid_index) + 0.5) * voxel_size
        grid_index = np.floor((center - grid_min) / voxel_size).astype(int)

        if np.all(grid_index >= 0) and np.all(grid_index < grid_shape):
            surface[tuple(grid_index)] = True

    return surface


def get_solid_grid(mesh, grid_min, grid_shape, voxel_size, ray_samples):
    if not mesh.is_watertight():
        raise RuntimeError(
            "Mesh is not watertight after duplicate vertices were merged. "
            "Repair the open scan gaps before calculating volume."
        )

    tensor_mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(tensor_mesh)

    grid_cells = int(np.prod(grid_shape, dtype=np.int64))
    solid_flat = np.zeros(grid_cells, dtype=bool)
    cells_per_x = int(grid_shape[1] * grid_shape[2])
    chunk_size = 500000

    for start in range(0, grid_cells, chunk_size):
        stop = min(start + chunk_size, grid_cells)
        flat_indices = np.arange(start, stop, dtype=np.int64)

        x_indices = flat_indices // cells_per_x
        remainder = flat_indices % cells_per_x
        y_indices = remainder // grid_shape[2]
        z_indices = remainder % grid_shape[2]

        grid_indices = np.column_stack((x_indices, y_indices, z_indices))
        points = grid_min + (grid_indices + 0.5) * voxel_size

        occupancy = scene.compute_occupancy(
            o3d.core.Tensor(points.astype(np.float32)),
            nsamples=ray_samples
        ).numpy()

        solid_flat[start:stop] = occupancy > 0.5

    return solid_flat.reshape(tuple(grid_shape))


def grid_to_point_cloud(mask, grid_min, voxel_size, color):
    grid_indices = np.argwhere(mask)
    point_cloud = o3d.geometry.PointCloud()

    if len(grid_indices) == 0:
        return point_cloud

    points = grid_min + (grid_indices + 0.5) * voxel_size
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.paint_uniform_color(color)
    return point_cloud


def volume_to_cm3(volume, units):
    if units == "m":
        return volume * 1000000
    if units == "mm":
        return volume / 1000
    return volume


def calculate_results(
    healthy_solid,
    damaged_solid,
    healthy_surface,
    damaged_surface,
    healthy_interior,
    damaged_interior,
    voxel_size,
    units,
    ray_samples
):
    healthy_count = int(np.count_nonzero(healthy_solid))
    damaged_count = int(np.count_nonzero(damaged_solid))
    overlap_count = int(np.count_nonzero(healthy_solid & damaged_solid))
    missing_count = int(np.count_nonzero(healthy_solid & ~damaged_solid))
    extra_count = int(np.count_nonzero(damaged_solid & ~healthy_solid))

    voxel_volume = voxel_size ** 3
    union_count = healthy_count + damaged_count - overlap_count

    results = {
        "units": units,
        "voxel_size": voxel_size,
        "ray_samples": ray_samples,
        "voxel_volume_cm3": volume_to_cm3(voxel_volume, units),
        "healthy_surface_voxels": int(np.count_nonzero(healthy_surface)),
        "damaged_surface_voxels": int(np.count_nonzero(damaged_surface)),
        "healthy_interior_voxels": int(np.count_nonzero(healthy_interior)),
        "damaged_interior_voxels": int(np.count_nonzero(damaged_interior)),
        "healthy_solid_voxels": healthy_count,
        "damaged_solid_voxels": damaged_count,
        "overlap_voxels": overlap_count,
        "missing_voxels": missing_count,
        "damaged_only_voxels": extra_count,
        "healthy_volume_cm3": volume_to_cm3(healthy_count * voxel_volume, units),
        "damaged_volume_cm3": volume_to_cm3(damaged_count * voxel_volume, units),
        "missing_volume_cm3": volume_to_cm3(missing_count * voxel_volume, units),
        "damaged_only_volume_cm3": volume_to_cm3(extra_count * voxel_volume, units),
        "net_volume_difference_cm3": volume_to_cm3(
            (healthy_count - damaged_count) * voxel_volume,
            units
        ),
        "missing_percent": 100 * missing_count / healthy_count if healthy_count else 0,
        "damaged_only_percent": 100 * extra_count / healthy_count if healthy_count else 0,
        "overlap_percent": 100 * overlap_count / healthy_count if healthy_count else 0,
        "damaged_containment_percent": (
            100 * overlap_count / damaged_count if damaged_count else 0
        ),
        "intersection_over_union": overlap_count / union_count if union_count else 0
    }

    return results


def save_results(
    output_folder,
    results,
    healthy_mesh,
    damaged_mesh,
    missing_cloud,
    extra_cloud,
    overlap_cloud
):
    output_folder = Path(output_folder)
    output_folder.mkdir(parents=True, exist_ok=True)

    results["healthy_mesh_watertight"] = healthy_mesh.is_watertight()
    results["damaged_mesh_watertight"] = damaged_mesh.is_watertight()

    with open(output_folder / "voxel_volume_results.csv", "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["measurement", "value"])
        for name, value in results.items():
            writer.writerow([name, value])

    o3d.io.write_point_cloud(
        str(output_folder / "missing_volume_voxels.ply"),
        missing_cloud
    )
    o3d.io.write_point_cloud(
        str(output_folder / "damaged_only_voxels.ply"),
        extra_cloud
    )

    overlay_cloud = overlap_cloud + missing_cloud + extra_cloud
    o3d.io.write_point_cloud(
        str(output_folder / "voxel_overlay_colored.ply"),
        overlay_cloud
    )


def print_mesh_details(name, mesh):
    dimensions = mesh.get_axis_aligned_bounding_box().get_extent()
    print(f"{name} dimensions: {dimensions}")
    print(f"{name} watertight: {mesh.is_watertight()}")


def print_results(results):
    print()
    print("Voxel volume result")
    print(f"healthy volume:      {results['healthy_volume_cm3']:.3f} cm^3")
    print(f"damaged volume:      {results['damaged_volume_cm3']:.3f} cm^3")
    print(f"missing volume:      {results['missing_volume_cm3']:.3f} cm^3")
    print(f"damaged-only volume: {results['damaged_only_volume_cm3']:.3f} cm^3")
    print(f"net difference:      {results['net_volume_difference_cm3']:.3f} cm^3")
    print(f"missing percent:     {results['missing_percent']:.3f}%")
    print(f"overlap percent:     {results['overlap_percent']:.3f}%")
    print(f"damaged containment: {results['damaged_containment_percent']:.3f}%")
    print(f"intersection/union:  {results['intersection_over_union']:.4f}")


def print_warnings(results):
    if results["healthy_solid_voxels"] == 0:
        print()
        print("WARNING: no healthy-brick interior voxels were found.")

    if results["damaged_solid_voxels"] == 0:
        print()
        print("WARNING: no damaged-brick interior voxels were found.")

    if results["damaged_only_percent"] > LARGE_EXTRA_FRACTION * 100:
        print()
        print("WARNING: damaged-only volume is large.")
        print("Check ICP alignment, OBJ scale, and the selected voxel size.")

    if results["damaged_containment_percent"] < LOW_CONTAINMENT_FRACTION * 100:
        print()
        print("WARNING: damaged-brick containment is low.")
        print("The volume comparison should not be used until alignment is checked.")


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Estimate missing brick volume with solid voxel grids."
    )

    # Files
    parser.add_argument("--healthy", default="healthy_brick.obj")
    parser.add_argument("--damaged", default="aligned_source.obj")
    parser.add_argument("--out_dir", "--out-dir", default="voxel_volume_output")

    # Voxel settings
    parser.add_argument(
        "--voxel_size",
        "--voxel-size",
        type=float,
        default=DEFAULT_VOXEL_SIZE
    )
    parser.add_argument(
        "--ray_samples",
        "--ray-samples",
        type=int,
        default=DEFAULT_RAY_SAMPLES
    )
    parser.add_argument(
        "--units",
        choices=["m", "cm", "mm"],
        default="m"
    )

    # Display
    parser.add_argument("--visualize", action="store_true")

    return parser.parse_args()


def main():
    args = parse_arguments()

    if args.voxel_size <= 0:
        raise ValueError("voxel_size must be greater than zero")
    if args.ray_samples <= 0 or args.ray_samples % 2 == 0:
        raise ValueError("ray_samples must be a positive odd number")

    print("Loading meshes")
    healthy_mesh = read_mesh(args.healthy)
    damaged_mesh = read_mesh(args.damaged)

    print_mesh_details("healthy", healthy_mesh)
    print_mesh_details("damaged", damaged_mesh)

    print("Creating shared voxel grid")
    grid_min, grid_shape = get_shared_grid(
        healthy_mesh,
        damaged_mesh,
        args.voxel_size
    )
    print(f"grid shape: {grid_shape}")
    print(f"grid cells: {int(np.prod(grid_shape, dtype=np.int64)):,}")

    print("Voxelizing mesh surfaces")
    healthy_surface = mesh_surface_grid(
        healthy_mesh,
        grid_min,
        grid_shape,
        args.voxel_size
    )
    damaged_surface = mesh_surface_grid(
        damaged_mesh,
        grid_min,
        grid_shape,
        args.voxel_size
    )

    print("Classifying solid voxel interiors")
    healthy_solid = get_solid_grid(
        healthy_mesh,
        grid_min,
        grid_shape,
        args.voxel_size,
        args.ray_samples
    )
    damaged_solid = get_solid_grid(
        damaged_mesh,
        grid_min,
        grid_shape,
        args.voxel_size,
        args.ray_samples
    )
    healthy_interior = healthy_solid & ~healthy_surface
    damaged_interior = damaged_solid & ~damaged_surface

    missing_mask = healthy_solid & ~damaged_solid
    extra_mask = damaged_solid & ~healthy_solid
    overlap_mask = healthy_solid & damaged_solid

    results = calculate_results(
        healthy_solid=healthy_solid,
        damaged_solid=damaged_solid,
        healthy_surface=healthy_surface,
        damaged_surface=damaged_surface,
        healthy_interior=healthy_interior,
        damaged_interior=damaged_interior,
        voxel_size=args.voxel_size,
        units=args.units,
        ray_samples=args.ray_samples
    )

    print_results(results)
    print_warnings(results)

    print("Creating output point clouds")
    missing_cloud = grid_to_point_cloud(
        missing_mask,
        grid_min,
        args.voxel_size,
        [0.0, 0.2, 1.0]
    )
    extra_cloud = grid_to_point_cloud(
        extra_mask,
        grid_min,
        args.voxel_size,
        [1.0, 0.0, 0.0]
    )
    overlap_cloud = grid_to_point_cloud(
        overlap_mask,
        grid_min,
        args.voxel_size,
        [0.7, 0.7, 0.7]
    )

    print("Saving files")
    save_results(
        output_folder=args.out_dir,
        results=results,
        healthy_mesh=healthy_mesh,
        damaged_mesh=damaged_mesh,
        missing_cloud=missing_cloud,
        extra_cloud=extra_cloud,
        overlap_cloud=overlap_cloud
    )

    final_output_path = Path(args.out_dir).resolve()
    print(f"output: {final_output_path}")

    if args.visualize:
        o3d.visualization.draw_geometries(
            [overlap_cloud, missing_cloud, extra_cloud],
            window_name="brick voxel volume"
        )


if __name__ == "__main__":
    main()
