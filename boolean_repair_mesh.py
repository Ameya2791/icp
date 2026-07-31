"""Create a smooth missing-part surface with a direct mesh difference."""

from pathlib import Path
import argparse

import numpy as np
import open3d as o3d


DEFAULT_SMOOTHING_ITERATIONS = 4
MINIMUM_COMPONENT_FRACTION = 0.001


def read_mesh(file_path):
    mesh = o3d.io.read_triangle_mesh(str(file_path))

    if mesh.is_empty():
        raise RuntimeError(f"Could not load mesh: {file_path}")

    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    return mesh


def remove_small_components(mesh):
    labels, triangle_counts, _ = mesh.cluster_connected_triangles()
    labels = np.asarray(labels)
    triangle_counts = np.asarray(triangle_counts)

    if len(triangle_counts) <= 1:
        return mesh, len(triangle_counts)

    minimum_count = max(
        20,
        int(len(mesh.triangles) * MINIMUM_COMPONENT_FRACTION),
    )
    keep_labels = np.flatnonzero(triangle_counts >= minimum_count)
    remove_mask = ~np.isin(labels, keep_labels)

    mesh.remove_triangles_by_mask(remove_mask)
    mesh.remove_unreferenced_vertices()
    return mesh, len(keep_labels)


def edge_summary(mesh):
    triangles = np.asarray(mesh.triangles)
    edges = np.vstack(
        (
            triangles[:, [0, 1]],
            triangles[:, [1, 2]],
            triangles[:, [2, 0]],
        )
    )
    edges.sort(axis=1)
    _, counts = np.unique(edges, axis=0, return_counts=True)
    return {
        "boundary": int(np.count_nonzero(counts == 1)),
        "manifold": int(np.count_nonzero(counts == 2)),
        "non_manifold": int(np.count_nonzero(counts > 2)),
    }


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Subtract an aligned damaged mesh from a healthy mesh."
    )
    parser.add_argument("--healthy", required=True)
    parser.add_argument("--damaged", required=True)
    parser.add_argument(
        "--output",
        default="missing_repair_mesh_boolean_clean.obj",
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=DEFAULT_SMOOTHING_ITERATIONS,
    )
    parser.add_argument("--tolerance", type=float, default=0.000001)
    return parser.parse_args()


def main():
    args = parse_arguments()

    if args.iterations < 0:
        raise ValueError("iterations cannot be negative")
    if args.tolerance <= 0:
        raise ValueError("tolerance must be greater than zero")

    print("Loading meshes")
    healthy_mesh = read_mesh(args.healthy)
    damaged_mesh = read_mesh(args.damaged)

    if not healthy_mesh.is_watertight():
        raise RuntimeError("The healthy mesh must be watertight")
    if not damaged_mesh.is_watertight():
        raise RuntimeError("The damaged mesh must be watertight")

    print("Calculating healthy minus damaged geometry")
    healthy_tensor = o3d.t.geometry.TriangleMesh.from_legacy(healthy_mesh)
    damaged_tensor = o3d.t.geometry.TriangleMesh.from_legacy(damaged_mesh)
    difference_tensor = healthy_tensor.boolean_difference(
        damaged_tensor,
        tolerance=args.tolerance,
    )
    difference_mesh = difference_tensor.to_legacy()

    difference_mesh.remove_duplicated_vertices()
    difference_mesh.remove_duplicated_triangles()
    difference_mesh.remove_degenerate_triangles()
    difference_mesh.remove_unreferenced_vertices()
    difference_mesh, component_count = remove_small_components(difference_mesh)

    if args.iterations:
        difference_mesh = difference_mesh.filter_smooth_taubin(
            number_of_iterations=args.iterations
        )

    difference_mesh.compute_vertex_normals()
    edge_counts = edge_summary(difference_mesh)

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if not o3d.io.write_triangle_mesh(
        str(output_path),
        difference_mesh,
        write_vertex_normals=False,
        write_triangle_uvs=False,
    ):
        raise RuntimeError(f"Could not save mesh: {output_path}")

    print(f"vertices:             {len(difference_mesh.vertices):,}")
    print(f"triangles:            {len(difference_mesh.triangles):,}")
    print(f"retained components:  {component_count}")
    print(f"boundary edges:       {edge_counts['boundary']:,}")
    print(f"non-manifold edges:   {edge_counts['non_manifold']:,}")
    print(f"output: {output_path.resolve()}")
    print()
    print("Use this OBJ for presentation. Use the voxel result for volume.")


if __name__ == "__main__":
    main()
