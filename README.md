# LiDAR-Based Alignment and Reconstruction of Damaged Bricks

This repository contains a Python workflow for comparing damaged brick scans
with an intact reference brick. The meshes are captured with Polycam Object
Mode and exported as OBJ files.

The workflow has four main stages:

1. Correct the physical scale of each Polycam mesh.
2. Align the damaged mesh with the healthy reference using Iterative Closest
   Point (ICP).
3. Measure missing material with a shared solid voxel grid.
4. Generate a smoother missing-piece surface with Boolean mesh subtraction.

The voxel and Boolean outputs have different purposes. Voxel analysis provides
the quantitative missing-volume result. Boolean subtraction provides a smoother
surface model for visualization or fabrication.

## Repository Layout

```text
icp/
|-- align_icp.py
|-- boolean_repair_mesh.py
|-- scale_brick_meshes.py
|-- voxel_volume_analysis.py
|-- requirements.txt
|-- LICENSE.md
`-- README.md
```

Original OBJ scans and generated outputs do not need to be stored in the
repository. Large meshes can be supplied separately or managed with Git LFS.

## Dependencies

The scripts require Python 3 and the following packages:

```text
numpy
open3d
```

Install them with:

```powershell
python -m pip install -r requirements.txt
```

## Input Files

The commands below assume this folder structure:

```text
LiDAR Project/
|-- icp/
|   |-- align_icp.py
|   |-- boolean_repair_mesh.py
|   |-- scale_brick_meshes.py
|   `-- voxel_volume_analysis.py
`-- meshes/
    |-- healthy_brick.obj
    |-- damaged_brick_1.obj
    |-- damaged_brick_2.obj
    `-- damaged_brick_3.obj
```

Run all commands from the `LiDAR Project` folder.

## 1. Scale the Polycam Meshes

Polycam did not assign exactly the same scale to every scan. Scale factors were
calculated by comparing oriented mesh dimensions with physical measurements.

```text
healthy brick:   0.280300
damaged brick 1: 0.289638
damaged brick 2: 0.317342
damaged brick 3: 0.282977
```

The factors are stored in `scale_brick_meshes.py`.

```powershell
python .\icp\scale_brick_meshes.py `
  --input_dir .\meshes `
  --out_dir .\meshes\scaled
```

The original OBJ files are not modified. The script creates scaled copies and
writes `mesh_scale_factors.csv` containing the applied factors and final mesh
dimensions.

## 2. Run ICP Alignment

The damaged brick is the source mesh and the healthy brick is the target. The
rigid transformation is

```text
p_aligned = R p_source + t
```

where `R` is the rotation matrix and `t` is the translation vector.

Example for damaged brick 1:

```powershell
python .\icp\align_icp.py `
  --source .\meshes\scaled\damaged_brick_1.obj `
  --target .\meshes\scaled\healthy_brick.obj `
  --out_dir .\icp\outputs\scaled\damaged_brick_1\icp `
  --points 50000 `
  --voxel_size 0.002 `
  --icp_threshold 0.012 `
  --diff_threshold 0.004 `
  --visualize
```

For another specimen, change the source filename and output directory.

### ICP Arguments

```text
--source          damaged-brick OBJ file
--target          healthy-reference OBJ file
--out_dir         output directory
--points          number of surface points sampled from each mesh
--voxel_size      downsampling size used during ICP
--icp_threshold   maximum ICP correspondence distance
--diff_threshold  distance used to classify unmatched surfaces
--visualize       display the colored Open3D overlay
```

`icp_threshold` determines which point pairs can be used during registration.
`diff_threshold` determines which aligned points are classified as unmatched.
Both values use the same units as the scaled meshes.

### ICP Outputs

```text
aligned_source.obj
aligned_source.ply
missing_piece_target_only.ply
source_only_extra.ply
overlay_colored.ply
transformation_matrix.txt
```

The ICP overlay colors are:

```text
green = healthy reference
gray  = aligned damaged brick
blue  = unmatched healthy-reference points
red   = damaged-only points
```

The unmatched-point percentage is a surface comparison. It is not a physical
missing-volume percentage.

## 3. Estimate Missing Volume

`voxel_volume_analysis.py` places both aligned meshes on the same grid, tests
voxel centers against the closed surfaces, and treats occupied voxels as solid
volume. For cubic voxels with side length `s`, missing volume is calculated as

```text
V_missing = N_missing s^3
```

where `N_missing` is the number of healthy-only voxels.

```powershell
python .\icp\voxel_volume_analysis.py `
  --healthy .\meshes\scaled\healthy_brick.obj `
  --damaged .\icp\outputs\scaled\damaged_brick_1\icp\aligned_source.obj `
  --out_dir .\icp\outputs\scaled\damaged_brick_1\voxel_volume `
  --voxel_size 0.002 `
  --ray_samples 3 `
  --mesh_step 1 `
  --units m `
  --visualize
```

### Volume Arguments

```text
--healthy       scaled healthy-reference mesh
--damaged       scaled and ICP-aligned damaged mesh
--out_dir       output directory
--voxel_size    side length of each voxel
--ray_samples   odd number of occupancy rays used per voxel
--mesh_step     repair-mesh resolution multiplier
--units         mesh units: m, cm, or mm
--visualize     display the colored voxel comparison
```

### Volume Outputs

```text
voxel_volume_results.csv
missing_volume_voxels.ply
missing_repair_mesh.obj
damaged_only_voxels.ply
voxel_overlay_colored.ply
```

`missing_repair_mesh.obj` is generated from the largest face-connected missing
region. The PLY output retains every detected region, including small isolated
regions.

Use `--mesh_step 1` for final output. A value of `2` creates a faster preview
mesh but does not alter the numerical missing-volume calculation. The CSV
reports healthy volume, damaged volume, estimated missing volume, overlap,
containment, damaged-only volume, and intersection-over-union.

The voxel overlay colors are:

```text
gray = volume shared by both bricks
blue = healthy-only volume, interpreted as missing material
red  = damaged-only volume
```

## 4. Generate the Boolean Repair Surface

The Boolean script calculates the geometric set difference

```text
R_boolean = H \ T(D)
```

where `H` is the healthy mesh and `T(D)` is the ICP-aligned damaged mesh.

```powershell
python .\icp\boolean_repair_mesh.py `
  --healthy .\meshes\scaled\healthy_brick.obj `
  --damaged .\icp\outputs\scaled\damaged_brick_1\icp\aligned_source.obj `
  --output .\icp\outputs\scaled\damaged_brick_1\voxel_volume\missing_repair_mesh_boolean.obj `
  --iterations 0 `
  --tolerance 0.000001
```

`--iterations 0` preserves the raw Boolean geometry. Increasing the iteration
count applies Taubin smoothing, which can improve presentation quality but
slightly changes the surface.

Both input meshes must be watertight. The script reports retained components,
boundary edges, and non-manifold edges so the result can be checked before use.
Nearly coincident or noisy surfaces may still produce small defects.

The Boolean OBJ follows the original scan triangles and is intended as the
smoother reconstructed surface. Missing-volume values and cost calculations
should continue to use `voxel_volume_results.csv`, not the Boolean mesh volume.

## Result Checks

Inspect each result before using it in an analysis:

- ICP fitness and RMSE should indicate stable registration.
- Intact regions should overlap in the ICP visualization.
- The aligned damaged mesh should be contained by the healthy reference.
- Damaged-only volume should be small relative to healthy volume.
- Voxel results should be checked at more than one voxel size.
- Digital volume should be compared with water-displacement measurements.
- Boolean output should be checked for detached components, boundary edges,
  and non-manifold edges.

The dominant sources of uncertainty are scan quality, physical scale, and ICP
alignment. Sampling more points from the same mesh does not create new surface
information.

## Files Excluded From Git

Generated outputs and Python cache files should not be committed:

```text
outputs/
__pycache__/
*.pyc
*.ply
```

OBJ scans should only be committed when intentionally included as sample data.

## License

This project is distributed under the GNU General Public License v3.0. See
`LICENSE.md` for the full license text.
