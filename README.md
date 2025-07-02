This project consists of two tasks:

1. **Task 1: Implementation of the ICP (Iterative Closest Point) algorithm with known and unknown correspondences**
2. **Task 2: Occupancy Grid mapping configuration (YAML file)**

---

## Directory Structure

.
├── task1.py             # Python script implementing the ICP algorithm  
├── task2.yaml           # YAML configuration file for occupancy grid mapping  
├── requirements.txt     # List of dependencies  
└── student_data/        # Sample point cloud data  
    └── student_data_3/  
        ├── bunny1.ply  
        └── bunny2.ply  

---

## Dependencies

- Python 3.7+
- numpy
- Open3D (`open3d`)
- PyYAML (optional, for reading `task2.yaml` in Python)

Install all dependencies with:

```bash
pip install -r requirements.txt
```

Example `requirements.txt`:

```
numpy
open3d
pyyaml
```

---

## Task 1: ICP Algorithm Usage

The `task1.py` script provides two variants of ICP:

- **Known Correspondences**  
  Function: `solve_icp_with_known_correspondence(point_cloud1, point_cloud2)`  
  Computes the rigid transform using SVD given point-to-point correspondences.

- **Unknown Correspondences**  
  Function: `solve_icp_without_known_correspondence(point_cloud1, point_cloud2, n_iter, threshold)`  
  Iteratively matches nearest neighbors and refines the transform until convergence or `n_iter` iterations.

### How to Run

Inside `task1.py`, uncomment the desired section in `main()`:

```python
# Known correspondences:
# solve_icp_with_known_correspondence(points1, points2)

# Unknown correspondences:
solve_icp_without_known_correspondence(points1, points2,
                                      n_iter=1000,
                                      threshold=0.01597565)
```

Then execute:

```bash
python task1.py
```

**Tips**:  
- Adjust file paths in `o3d.io.read_point_cloud(...)` to your data.  
- Tweak `n_iter` and `threshold` based on noise and overlap.

---

## Task 2: Occupancy Grid Configuration

The `task2.yaml` file defines parameters for an occupancy grid, commonly used in ROS navigation or custom mapping.

```yaml
image: task2.pgm            # Grid map image file
resolution: 0.050000        # Meters per pixel
origin: [-2.700097, -34.242831, 0.000000]  # World frame origin [x, y, yaw]
negate: 0                   # Invert colors? (0: no, 1: yes)
occupied_thresh: 0.65       # Threshold for occupied cells
free_thresh: 0.196          # Threshold for free cells
```

### Usage Examples

- **ROS**: Place `task2.yaml` under your `config` folder and reference in launch file:

  ```xml
  <param name="yaml_filename" value="$(find your_package)/config/task2.yaml" />
  ```

- **Python Script**: Load YAML with PyYAML and image with OpenCV or PIL to build the grid map.

---

