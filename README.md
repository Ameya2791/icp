# LiDAR Brick Alignment and Volume Analysis

This repository contains a Python workflow for comparing damaged brick scans
with an intact reference brick. The scans are captured in Polycam Object Mode
and exported as OBJ meshes.

The workflow has three stages:

1. Scale each Polycam mesh to measured physical dimensions.
2. Align a damaged brick with the healthy reference using Iterative Closest
   Point (ICP).
3. Estimate missing volume with a shared solid voxel grid.

## Repository Layout

```text
icp/
|-- align_icp.py
|-- scale_brick_meshes.py
|-- voxel_volume_analysis.py
|-- requirements.txt
`-- README.md
```

The original OBJ scans and generated output files are not required in the
repository. Large meshes can be supplied separately or managed with Git LFS.

## Dependencies

The scripts use:

```text
numpy
open3d
```

Install the dependencies with:

```powershell
python -m pip install -r requirements.txt
```

## Input Files

The examples below assume this folder structure:

```text
LiDAR Project/
|-- icp/
|   |-- align_icp.py
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

Polycam did not assign exactly the same scale to every scan. The scale factors
were calculated by comparing oriented mesh dimensions with physical brick
measurements.

```text
healthy brick:   0.280300
damaged brick 1: 0.289638
damaged brick 2: 0.317342
damaged brick 3: 0.282977
```

The factors are stored in `scale_brick_meshes.py`. Run:

```powershell
python .\icp\scale_brick_meshes.py `
  --input_dir .\meshes `
  --out_dir .\meshes\scaled
```

The script leaves the original OBJ files unchanged. It creates scaled copies
and writes `mesh_scale_factors.csv` with the applied factors and final mesh
dimensions.

## 2. Run ICP Alignment

The damaged brick is the source and the healthy brick is the target. Always use
the scaled meshes for the volume-analysis workflow.

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

For another specimen, change the source filename and output folder.

### ICP Arguments

```text
--source          damaged brick OBJ file
--target          healthy reference OBJ file
--out_dir         folder where outputs are saved
--points          number of mesh surface points to sample
--voxel_size      voxel size used for ICP downsampling
--icp_threshold   maximum ICP correspondence distance
--diff_threshold  distance used to classify unmatched surface points
--visualize       show the colored Open3D overlay
```

`icp_threshold` controls which point pairs can be used during registration.
`diff_threshold` controls which aligned surface points are classified as
unmatched. Both thresholds use the same units as the scaled meshes.

### ICP Outputs

```text
aligned_source.obj
aligned_source.ply
missing_piece_target_only.ply
source_only_extra.ply
overlay_colored.ply
transformation_matrix.txt
```

The overlay colors are:

```text
green = healthy reference
gray  = aligned damaged brick
blue  = unmatched healthy-reference surface points
red   = damaged-only points
```

The unmatched-point percentage is a surface comparison. It should not be
reported as physical missing-volume percentage.

## 3. Estimate Missing Volume

`voxel_volume_analysis.py` places the healthy and aligned damaged meshes on the
same grid. It tests voxel centers against the closed mesh surfaces and treats
the occupied voxels as solid volume.

```powershell
python .\icp\voxel_volume_analysis.py `
  --healthy .\meshes\scaled\healthy_brick.obj `
  --damaged .\icp\outputs\scaled\damaged_brick_1\icp\aligned_source.obj `
  --out_dir .\icp\outputs\scaled\damaged_brick_1\voxel_volume `
  --voxel_size 0.002 `
  --ray_samples 3 `
  --units m `
  --visualize
```

### Volume Arguments

```text
--healthy       physically scaled healthy reference mesh
--damaged       physically scaled and ICP-aligned damaged mesh
--out_dir       folder where volume outputs are saved
--voxel_size    side length of each voxel
--ray_samples   odd number of occupancy rays used per voxel
--units         mesh units: m, cm, or mm
--visualize     show the colored voxel comparison
```

### Volume Outputs

```text
voxel_volume_results.csv
missing_volume_voxels.ply
damaged_only_voxels.ply
voxel_overlay_colored.ply
```

The volume overlay colors are:

```text
gray = volume shared by both bricks
blue = healthy-only volume, interpreted as missing material
red  = damaged-only volume
```

`voxel_volume_results.csv` reports healthy volume, damaged volume, estimated
missing volume, damaged-only volume, overlap, containment, and
intersection-over-union.

## Result Checks

A volume result should be inspected before it is used:

- ICP fitness and RMSE should indicate stable registration.
- Intact regions should overlap in the ICP visualization.
- Damaged-brick containment should be high.
- Damaged-only volume should be small relative to the healthy volume.
- Results should be checked at more than one voxel size.
- Digital volume should be compared with a physical measurement such as water
  displacement.

The voxel method requires closed meshes. The script stops if a mesh remains
non-watertight after duplicate vertices are merged.

## Files Excluded From Git

Generated outputs and Python cache files should not be committed:

```text
outputs/
__pycache__/
*.pyc
*.ply
```

OBJ scans should only be committed when they are intentionally included as
small sample data.
