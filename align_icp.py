r"""
Align a damaged brick scan to a healthy brick scan using ICP.

The damaged brick is used as the source cloud. The healthy brick is used as
the target cloud. After registration, target points that do not have a close
nearest-neighbor match in the aligned source cloud are saved as the missing
brick region.

The script is intended for the LiDAR brick-repair workflow:

1. Scan an intact reference (healthy) brick and one damaged brick.
2. Clean both scans in Blender or MeshLab.
3. Export both cleaned scans as OBJ meshes.
4. Run this script to align the damaged mesh to the reference mesh.
5. Use the file missing_piece_target_only.ply for surface reconstruction.

The default values assume the scan units are meters. If the OBJ files were
exported in millimeters, the voxel and distance thresholds should usually be
scaled up by 1000.

Inputs
------
source:
    Damaged or broken brick mesh.

target:
    Healthy or complete reference brick mesh.

Outputs
-------
aligned_source.obj:
    Damaged mesh after ICP alignment.

aligned_source.ply:
    Sampled damaged point cloud after ICP alignment.

missing_piece_target_only.ply:
    Points on the healthy brick that are not closely matched by the damaged
    brick. This is the main output for the missing-piece reconstruction step.

source_only_extra.ply:
    Points on the damaged scan that do not match the healthy reference. This
    is primarily useful for checking scan artifacts, loose fragments, or bad ICP runs.

overlay_colored.ply:
    Colored point-cloud overlay for visual checking.

transformation_matrix.txt:
    ICP transformation matrix applied to the damaged scan.

Folder Layout
-------------
LiDAR Project/
    icp/
        align_icp.py
    meshes/
        healthy_brick.obj
        damaged_brick_1.obj
        damaged_brick_2.obj

Example
-------
python .\icp\align_icp.py \
    --source .\meshes\damaged_brick_1.obj \
    --target .\meshes\healthy_brick.obj \
    --out_dir .\icp\outputs\damaged_brick_1 \
    --visualize
"""

from pathlib import Path
import argparse

import numpy as np
import open3d as o3d


DEFAULT_POINTS = 50000
DEFAULT_VOXEL_SIZE = 0.005
DEFAULT_ICP_THRESHOLD = 0.02
DEFAULT_DIFF_THRESHOLD = 0.0025
LARGE_MISSING_FRACTION = 0.35


def read_mesh(file_path):
    file_path = Path(file_path)

    mesh = o3d.io.read_triangle_mesh(str(file_path))

    if mesh.is_empty():
        raise RuntimeError(f"Could not load mesh, or mesh is empty: {file_path}")

    mesh.compute_vertex_normals()
    return mesh


def sample_mesh(mesh, num_points):
    sampled_cloud = mesh.sample_points_uniformly(number_of_points=num_points)
    return sampled_cloud


def downsample_and_estimate_normals(point_cloud, voxel_size):
    small_cloud = point_cloud.voxel_down_sample(voxel_size)

    normal_search_radius = voxel_size * 2

    small_cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=normal_search_radius,
            max_nn=30
        )
    )

    return small_cloud


def run_icp(source_down, target_down, threshold):
    initial_guess = np.eye(4)

    registration_result = o3d.pipelines.registration.registration_icp(
        source_down,
        target_down,
        threshold,
        initial_guess,
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )

    return registration_result


def get_unmatched_points(base_cloud, compare_to_cloud, distance_threshold):
    distances = np.asarray(base_cloud.compute_point_cloud_distance(compare_to_cloud))

    unmatched_ids = []
    for point_index, distance in enumerate(distances):
        if distance > distance_threshold:
            unmatched_ids.append(point_index)

    return base_cloud.select_by_index(unmatched_ids)


def color_copy(point_cloud, rgb):
    cloud_copy = o3d.geometry.PointCloud(point_cloud)
    cloud_copy.paint_uniform_color(rgb)
    return cloud_copy


def save_results(
    output_folder,
    aligned_mesh,
    aligned_source_cloud,
    target_cloud,
    missing_piece,
    source_extra,
    transform_matrix
):
    output_folder = Path(output_folder)
    output_folder.mkdir(parents=True, exist_ok=True)

    o3d.io.write_triangle_mesh(
        str(output_folder / "aligned_source.obj"),
        aligned_mesh
    )

    o3d.io.write_point_cloud(
        str(output_folder / "aligned_source.ply"),
        aligned_source_cloud
    )

    # Main file for the MeshLab reconstruction step
    o3d.io.write_point_cloud(
        str(output_folder / "missing_piece_target_only.ply"),
        missing_piece
    )

    # Check file for scan fragments, noise, or poor registration
    o3d.io.write_point_cloud(
        str(output_folder / "source_only_extra.ply"),
        source_extra
    )

    # green = healthy reference
    # gray  = damaged brick after alignment
    # blue  = missing region from the target
    # red   = damaged-only extra points
    healthy_view = color_copy(target_cloud, [0.2, 0.8, 0.2])
    damaged_view = color_copy(aligned_source_cloud, [0.7, 0.7, 0.7])
    missing_view = color_copy(missing_piece, [0.0, 0.2, 1.0])
    extra_view = color_copy(source_extra, [1.0, 0.0, 0.0])

    overlay_cloud = healthy_view + damaged_view + missing_view + extra_view

    o3d.io.write_point_cloud(
        str(output_folder / "overlay_colored.ply"),
        overlay_cloud
    )

    np.savetxt(
        output_folder / "transformation_matrix.txt",
        transform_matrix,
        fmt="%.8f"
    )


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Align a damaged brick scan to a healthy reference scan."
    )

    # Files
    parser.add_argument("--source", default="source.obj")
    parser.add_argument("--target", default="target.obj")
    parser.add_argument("--out_dir", "--out-dir", default="icp_output")

    # ICP settings
    parser.add_argument("--points", type=int, default=DEFAULT_POINTS)
    parser.add_argument("--voxel_size", "--voxel-size", type=float, default=DEFAULT_VOXEL_SIZE)
    parser.add_argument("--icp_threshold", "--icp-threshold", type=float, default=DEFAULT_ICP_THRESHOLD)
    parser.add_argument("--diff_threshold", "--diff-threshold", type=float, default=DEFAULT_DIFF_THRESHOLD)

    # Display
    parser.add_argument("--visualize", action="store_true")

    return parser.parse_args()


def main():
    args = parse_arguments()

    print("Loading meshes")
    broken_mesh = read_mesh(args.source)
    healthy_mesh = read_mesh(args.target)

    print("Sampling mesh surfaces")
    broken_cloud = sample_mesh(broken_mesh, args.points)
    healthy_cloud = sample_mesh(healthy_mesh, args.points)

    print("Preparing ICP clouds")
    broken_down = downsample_and_estimate_normals(broken_cloud, args.voxel_size)
    healthy_down = downsample_and_estimate_normals(healthy_cloud, args.voxel_size)

    print("Running ICP")
    icp_result = run_icp(
        broken_down,
        healthy_down,
        args.icp_threshold
    )

    print()
    print("ICP result")
    print(f"fitness: {icp_result.fitness:.4f}")
    print(f"rmse:    {icp_result.inlier_rmse:.6f}")
    print("matrix:")
    print(icp_result.transformation)

    aligned_broken_cloud = o3d.geometry.PointCloud(broken_cloud)
    aligned_broken_cloud.transform(icp_result.transformation)

    aligned_broken_mesh = o3d.geometry.TriangleMesh(broken_mesh)
    aligned_broken_mesh.transform(icp_result.transformation)

    print("Finding unmatched reference points")
    missing_piece = get_unmatched_points(
        base_cloud=healthy_cloud,
        compare_to_cloud=aligned_broken_cloud,
        distance_threshold=args.diff_threshold
    )

    print("Finding damaged-only points")
    source_extra = get_unmatched_points(
        base_cloud=aligned_broken_cloud,
        compare_to_cloud=healthy_cloud,
        distance_threshold=args.diff_threshold
    )

    missing_count = len(missing_piece.points)
    extra_count = len(source_extra.points)
    target_count = len(healthy_cloud.points)

    print(f"missing-piece points: {missing_count}")
    print(f"source-only points:   {extra_count}")

    if target_count > 0 and missing_count / target_count > LARGE_MISSING_FRACTION:
        print()
        print("WARNING: missing-piece point count is very large.")
        print("This can mean ICP alignment failed, source/target were swapped,")
        print("the OBJ units need larger thresholds, or the scans are at different scales.")

    print("Saving files")
    save_results(
        output_folder=args.out_dir,
        aligned_mesh=aligned_broken_mesh,
        aligned_source_cloud=aligned_broken_cloud,
        target_cloud=healthy_cloud,
        missing_piece=missing_piece,
        source_extra=source_extra,
        transform_matrix=icp_result.transformation
    )

    final_output_path = Path(args.out_dir).resolve()
    print(f"output: {final_output_path}")

    if args.visualize:
        o3d.visualization.draw_geometries(
            [
                color_copy(healthy_cloud, [0.2, 0.8, 0.2]),
                color_copy(aligned_broken_cloud, [0.7, 0.7, 0.7]),
                color_copy(missing_piece, [0.0, 0.2, 1.0]),
                color_copy(source_extra, [1.0, 0.0, 0.0])
            ],
            window_name="brick ICP"
        )


if __name__ == "__main__":
    main()
