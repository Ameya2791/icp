r"""
Scale the Polycam brick meshes to measured physical dimensions.

Each Polycam scan has its own scale factor. This script applies the factors
calculated from the measured brick dimensions and saves new OBJ files for the
ICP and voxel-volume steps. The original OBJ files are not changed.

Inputs
------
input_dir:
    Folder containing the original healthy and damaged OBJ meshes.

out_dir:
    Folder where the physically scaled meshes will be saved.

Outputs
-------
healthy_brick.obj:
    Scaled healthy reference mesh.

damaged_brick_1.obj, damaged_brick_2.obj, damaged_brick_3.obj:
    Scaled damaged meshes.

mesh_scale_factors.csv:
    Applied scale factors and final oriented bounding-box dimensions.

Example
-------
python .\icp\scale_brick_meshes.py \
    --input_dir .\meshes \
    --out_dir .\meshes\scaled
"""

from pathlib import Path
import argparse
import csv

import numpy as np
import open3d as o3d


BRICK_SCALE_FACTORS = {
    "healthy_brick.obj": 0.280300,
    "damaged_brick_1.obj": 0.289638,
    "damaged_brick_2.obj": 0.317342,
    "damaged_brick_3.obj": 0.282977
}


def read_mesh(file_path):
    file_path = Path(file_path)
    mesh = o3d.io.read_triangle_mesh(str(file_path), enable_post_processing=False)

    if mesh.is_empty():
        raise RuntimeError(f"Could not load mesh, or mesh is empty: {file_path}")

    mesh.compute_vertex_normals()
    return mesh


def get_dimensions_mm(mesh):
    bounding_box = mesh.get_minimal_oriented_bounding_box()
    dimensions_m = np.sort(np.asarray(bounding_box.extent))[::-1]
    return dimensions_m * 1000


def scale_mesh(mesh, scale_factor):
    scaled_mesh = o3d.geometry.TriangleMesh(mesh)
    scaled_mesh.scale(scale_factor, center=[0, 0, 0])

    # Texture files are not needed for the geometry analysis.
    scaled_mesh.triangle_uvs = o3d.utility.Vector2dVector()
    scaled_mesh.triangle_material_ids = o3d.utility.IntVector()
    scaled_mesh.textures = []

    return scaled_mesh


def save_scale_report(output_folder, rows):
    report_path = Path(output_folder) / "mesh_scale_factors.csv"

    with open(report_path, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "mesh",
                "scale_factor",
                "scaled_length_mm",
                "scaled_width_mm",
                "scaled_height_mm"
            ]
        )
        writer.writerows(rows)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Scale Polycam brick meshes to measured physical dimensions."
    )

    parser.add_argument("--input_dir", "--input-dir", default="meshes")
    parser.add_argument("--out_dir", "--out-dir", default="meshes/scaled")

    return parser.parse_args()


def main():
    args = parse_arguments()

    input_folder = Path(args.input_dir)
    output_folder = Path(args.out_dir)
    output_folder.mkdir(parents=True, exist_ok=True)

    report_rows = []

    for file_name, scale_factor in BRICK_SCALE_FACTORS.items():
        input_path = input_folder / file_name
        output_path = output_folder / file_name

        print(f"Loading {file_name}")
        mesh = read_mesh(input_path)

        print(f"Applying scale factor {scale_factor:.6f}")
        scaled_mesh = scale_mesh(mesh, scale_factor)
        dimensions_mm = get_dimensions_mm(scaled_mesh)

        print(
            "scaled dimensions: "
            f"{dimensions_mm[0]:.2f} x "
            f"{dimensions_mm[1]:.2f} x "
            f"{dimensions_mm[2]:.2f} mm"
        )

        if not o3d.io.write_triangle_mesh(
            str(output_path),
            scaled_mesh,
            write_vertex_normals=False,
            write_triangle_uvs=False
        ):
            raise RuntimeError(f"Could not save scaled mesh: {output_path}")

        report_rows.append(
            [
                file_name,
                scale_factor,
                dimensions_mm[0],
                dimensions_mm[1],
                dimensions_mm[2]
            ]
        )

    save_scale_report(output_folder, report_rows)
    print(f"output: {output_folder.resolve()}")


if __name__ == "__main__":
    main()
