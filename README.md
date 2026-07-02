# LiDAR Brick ICP Alignment

This repository contains a Python workflow for aligning damaged brick scans to an intact reference brick scan using Iterative Closest Point (ICP). The workflow was developed for LiDAR-based brick damage analysis using OBJ meshes exported from Polycam Object Mode.

The main script is `align_icp.py`. It aligns a damaged brick scan to a healthy reference brick scan, then extracts healthy-reference points that are not matched by the damaged scan. These unmatched reference points represent missing material.

## Main Idea

The damaged brick is always the source. The healthy brick is always the target.

After ICP alignment, the script computes nearest-neighbor distances from the healthy reference point cloud to the aligned damaged point cloud. If a healthy-reference point is farther than the selected difference threshold, it is saved as missing material.

## Repository Layout

```text
icp/
|-- align_icp.py
|-- requirements.txt
|-- README.md
`-- meshes/                         optional sample data
    |-- healthy_brick.obj
    |-- damaged_brick_1.obj
    `-- damaged_brick_2.obj
```

If mesh files are too large for GitHub, keep them outside the repository or use Git LFS.

## Dependencies

The project uses:

```text
numpy
open3d
```

Install dependencies with:

```powershell
python -m pip install -r requirements.txt
```

Check that Open3D is installed:

```powershell
python -c "import open3d as o3d; import numpy as np; print(o3d.__version__)"
```

## Running the ICP Script

Run commands from the project folder that contains `icp/` and `meshes/`.

Example for damaged brick 1:

```powershell
python .\icp\align_icp.py `
  --source .\meshes\damaged_brick_1.obj `
  --target .\meshes\healthy_brick.obj `
  --out_dir .\icp\outputs\damaged_brick_1 `
  --icp_threshold 0.04 `
  --diff_threshold 0.015 `
  --visualize
```

Example for damaged brick 2:

```powershell
python .\icp\align_icp.py `
  --source .\meshes\damaged_brick_2.obj `
  --target .\meshes\healthy_brick.obj `
  --out_dir .\icp\outputs\damaged_brick_2 `
  --icp_threshold 0.04 `
  --diff_threshold 0.015 `
  --visualize
```

The exact threshold values should be tuned for the scan quality and amount of material loss.

## Script Arguments

```text
--source          damaged brick OBJ file
--target          healthy reference OBJ file
--out_dir         folder where outputs are saved
--points          number of mesh surface points to sample
--voxel_size      voxel size for downsampling
--icp_threshold   correspondence distance used during ICP
--diff_threshold  distance used to classify missing/material-loss points
--visualize       show the colored Open3D overlay
```

The script also accepts hyphenated versions of selected options, such as `--out-dir`, `--voxel-size`, `--icp-threshold`, and `--diff-threshold`.

## Threshold Guidance

`icp_threshold` controls how far apart points can be during ICP correspondence matching.

```text
smaller value = stricter alignment correspondences
larger value  = more permissive alignment correspondences
```

`diff_threshold` controls what is classified as missing material after alignment.

```text
smaller value = more healthy-reference points marked as missing
larger value  = fewer healthy-reference points marked as missing
```

For meter-scale OBJ files, useful starting values are:

```text
icp_threshold:  0.04 to 0.08
diff_threshold: 0.010 to 0.030
```

If the OBJ files are exported in millimeters instead of meters, scale the distance parameters by 1000:

```text
--voxel_size 5 --icp_threshold 40 --diff_threshold 15
```

## Output Files

Each run writes the following files to `--out_dir`:

```text
aligned_source.obj
aligned_source.ply
missing_piece_target_only.ply
source_only_extra.ply
overlay_colored.ply
transformation_matrix.txt
```

`aligned_source.obj` is the damaged mesh after ICP alignment.

`aligned_source.ply` is the sampled damaged point cloud after ICP alignment.

`missing_piece_target_only.ply` is the main missing-material output. It contains points from the healthy reference brick that do not have a nearby match in the aligned damaged scan.

`source_only_extra.ply` contains damaged-scan points that do not match the healthy reference. These regions can indicate fracture roughness, scan noise, loose fragments, or alignment error.

`overlay_colored.ply` is a colored point-cloud overlay for visual inspection.

`transformation_matrix.txt` stores the ICP transformation matrix applied to the damaged scan.

## Overlay Color Key

```text
green = healthy reference brick
gray  = damaged brick after ICP alignment
blue  = missing/material-loss region from the healthy reference
red   = damaged-only extra points, fracture roughness, scan noise, or alignment error
```

A good alignment should show strong overlap between the green reference and gray damaged scan over intact regions. Blue should appear where material is absent from the damaged brick. Red should be limited to rough fractured areas or small unmatched regions.

If the damaged brick is smaller or eroded on many sides, the blue missing-material region may appear around multiple faces rather than as one localized chunk.

## Notes for GitHub

Do not commit generated outputs or Python cache files:

```text
outputs/
__pycache__/
*.pyc
*.ply
```

OBJ meshes can be committed only if they are small enough and intended as sample data. For large scan files, use Git LFS or provide the data separately.
