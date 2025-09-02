# Double-S (Jerk-limited) Velocity Planner

A small C++ implementation of a jerk-limited double-S motion planner for 1D (Cartesian axis) movement. Supports non-zero start/end velocities, and enforces maximum velocity, acceleration, and jerk.

## Build

```bash
cmake -S /workspace -B /workspace/build -DCMAKE_BUILD_TYPE=Release
cmake --build /workspace/build -j
```

## CLI Usage

```bash
/workspace/build/ds_preview s0 s1 v0 v1 v_max a_max j_max dt
/workspace/build/ds_preview --path w0,w1,... v_start v_end v_max a_max j_max dt
```

- **s0/s1**: start/target position
- **v0/v1**: start/end velocity
- **v_max**: maximum velocity
- **a_max**: maximum acceleration magnitude
- **j_max**: maximum jerk magnitude
- **dt**: sample interval (seconds) for CSV output

Outputs CSV with columns: t,s,v,a

### Example

```bash
/workspace/build/ds_preview 0 1 0 0 1.0 2.0 10.0 0.01 > traj.csv
 /workspace/build/ds_preview --path 0,0.3,0.8,1.2,2.0 0 0 1.0 2.0 10.0 0.01 > traj_path.csv
```

Open `traj.csv` to visualize or plot.

## Library API

See `src/ds_planner.h`:

- `ds::Profile ds::planDoubleS(const Boundary&, const MotionLimits&)` — compute the S-curve profile
- `ds::State ds::sample(const Profile&, double t)` — sample s, v, a at time t
 - `ds::Profile ds::planPath(const std::vector<double>&, double v_start, double v_end, const MotionLimits&)` — multi-point path with forward-backward lookahead

## Notes

- Planning is performed along a single axis; apply per-axis for Cartesian preview.
- If distance is insufficient to reach `v_max`, the planner computes a no-cruise profile with a reduced peak velocity.
- Throws on infeasible zero-distance with different velocities.