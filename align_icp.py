"""
align_icp.py

Aligns a damaged brick scan to a healthy reference brick scan using ICP.
After alignment, it extracts the part of the healthy brick that is missing
from the damaged brick.

source = damaged / broken brick
target = healthy / complete brick
"""

from pathlib import Path
import argparse

import numpy as np
import open3d as o3d


def read_mesh(path):
    """Load a mesh file and make sure it contains geometry."""
    mesh = o3d.io.read_triangle_mesh(path)

    if mesh.is_empty():
        raise RuntimeError(f"Could not load mesh: {path}")

    mesh.compute_vertex_normals()
    return mesh


def sample_mesh(mesh, num_points):
    """Convert a triangle mesh into a point cloud by sampling points on the surface."""
    return mesh.sample_points_uniformly(number_of_points=num_points)


def downsample_and_estimate_normals(pcd, voxel_size):
    """Downsample the point cloud and estimate normals for point-to-plane ICP."""
    down = pcd.voxel_down_sample(voxel_size)

    down.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * 2,
            max_nn=30
        )
    )

    return down


def run_icp(source_down, target_down, threshold):
    """Run ICP and return the alignment result."""
    start_transform = np.eye(4)

    result = o3d.pipelines.registration.registration_icp(
        source_down,
        target_down,
        threshold,
        start_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )

    return result


def get_unmatched_points(reference_pcd, comparison_pcd, distance_threshold):
    """
    Finds points in reference_pcd that are far away from comparison_pcd.

    For example:
    - healthy points far from the damaged brick = missing brick piece
    - damaged points far from the healthy brick = mis-scan or extra fragments
    """
    distances = np.asarray(reference_pcd.compute_point_cloud_distance(comparison_pcd))
    unmatched_indices = np.where(distances > distance_threshold)[0]

    return reference_pcd.select_by_index(unmatched_indices)


def color_copy(pcd, color):
    """Make a colored copy of a point cloud for easier visual."""
    copy = o3d.geometry.PointCloud(pcd)
    copy.paint_uniform_color(color)
    return copy


def save_results(out_dir, aligned_mesh, aligned_source, target_pcd,
                 missing_piece, source_extra, transform):
    """Save the files that are useful for checking and reconstruction."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    o3d.io.write_triangle_mesh(str(out_dir / "aligned_source.obj"), aligned_mesh)
    o3d.io.write_point_cloud(str(out_dir / "aligned_source.ply"), aligned_source)
    o3d.io.write_point_cloud(str(out_dir / "missing_piece_target_only.ply"), missing_piece)
    o3d.io.write_point_cloud(str(out_dir / "source_only_extra.ply"), source_extra)

    # Green = healthy reference
    # Gray = aligned damaged brick
    # Blue = missing piece from healthy brick
    # Red = damaged-only extra points/noise
    overlay = (
        color_copy(target_pcd, [0.2, 0.8, 0.2]) +
        color_copy(aligned_source, [0.7, 0.7, 0.7]) +
        color_copy(missing_piece, [0.0, 0.2, 1.0]) +
        color_copy(source_extra, [1.0, 0.0, 0.0])
    )

    o3d.io.write_point_cloud(str(out_dir / "overlay_colored.ply"), overlay)
    np.savetxt(out_dir / "transformation_matrix.txt", transform, fmt="%.8f")


def main():
    parser = argparse.ArgumentParser(
        description="Align a damaged brick to a healthy brick and extract missing geometry."
    )

    parser.add_argument("--source", default="source.obj",
                        help="Damaged brick mesh. Default: source.obj")
    parser.add_argument("--target", default="target.obj",
                        help="Healthy reference brick mesh. Default: target.obj")
    parser.add_argument("--out_dir", default="icp_output",
                        help="Folder where output files will be saved.")

    parser.add_argument("--points", type=int, default=50000,
                        help="Number of points sampled from each mesh.")
    parser.add_argument("--voxel_size", type=float, default=0.005,
                        help="Voxel size for downsampling. Default assumes meters.")
    parser.add_argument("--icp_threshold", type=float, default=0.02,
                        help="Max point distance used during ICP.")
    parser.add_argument("--diff_threshold", type=float, default=0.0025,
                        help="Distance used to decide which points are missing.")

    parser.add_argument("--visualize", action="store_true",
                        help="Show the aligned point clouds after processing.")

    args = parser.parse_args()

    print("Loading meshes...")
    source_mesh = read_mesh(args.source)
    target_mesh = read_mesh(args.target)

    print("Sampling meshes into point clouds...")
    source_pcd = sample_mesh(source_mesh, args.points)
    target_pcd = sample_mesh(target_mesh, args.points)

    print("Downsampling point clouds...")
    source_down = downsample_and_estimate_normals(source_pcd, args.voxel_size)
    target_down = downsample_and_estimate_normals(target_pcd, args.voxel_size)

    print("Running ICP alignment...")
    icp_result = run_icp(source_down, target_down, args.icp_threshold)

    print()
    print("ICP finished")
    print(f"Fitness: {icp_result.fitness:.4f}")
    print(f"RMSE:    {icp_result.inlier_rmse:.6f}")
    print("Transformation matrix:")
    print(icp_result.transformation)

    print("Applying transform to damaged brick...")
    aligned_source = o3d.geometry.PointCloud(source_pcd)
    aligned_source.transform(icp_result.transformation)

    aligned_mesh = o3d.geometry.TriangleMesh(source_mesh)
    aligned_mesh.transform(icp_result.transformation)

    print("Extracting missing geometry...")
    missing_piece = get_unmatched_points(
        reference_pcd=target_pcd,
        comparison_pcd=aligned_source,
        distance_threshold=args.diff_threshold
    )

    source_extra = get_unmatched_points(
        reference_pcd=aligned_source,
        comparison_pcd=target_pcd,
        distance_threshold=args.diff_threshold
    )

    print(f"Missing-piece points found: {len(missing_piece.points)}")
    print(f"Source-only extra points found: {len(source_extra.points)}")

    print("Saving results...")
    save_results(
        out_dir=args.out_dir,
        aligned_mesh=aligned_mesh,
        aligned_source=aligned_source,
        target_pcd=target_pcd,
        missing_piece=missing_piece,
        source_extra=source_extra,
        transform=icp_result.transformation
    )

    print(f"Done. Output saved to: {Path(args.out_dir).resolve()}")

    if args.visualize:
        o3d.visualization.draw_geometries(
            [
                color_copy(target_pcd, [0.2, 0.8, 0.2]),
                color_copy(aligned_source, [0.7, 0.7, 0.7]),
                color_copy(missing_piece, [0.0, 0.2, 1.0]),
                color_copy(source_extra, [1.0, 0.0, 0.0])
            ],
            window_name="ICP result: green=healthy, gray=damaged, blue=missing, red=extra"
        )


if __name__ == "__main__":
    main()
