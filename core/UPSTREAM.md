# Upstream integration

## Pin

- Source: `upstream/BIEVR-LIO-SLAM` (submodule, `https://github.com/S0UL4/BIEVR-LIO-SLAM.git`)
- Revision: `b45850589e58d63bf0877b2e09875413e7a0cef8` (`main`)
- Package: `BIEVR/` and `modules/`

The submodule stays unmodified. CMake copies `BIEVR/include`, `BIEVR/src`, and `modules`
into `build/upstream_staged`, then applies `patches/integration` with zero fuzz.
Patch failures indicate upstream drift.

## Bridge boundary

The bridge replaces upstream ROS interfaces with `cpp/bindings.cpp`, compiling the
staged estimator and modules directly into `bievr._core`.

## Configuration

The offline bridge loads upstream parameter and sensor YAML configs directly from
`upstream/BIEVR-LIO-SLAM/config/`.

## Patch series

| Patch | Upstream file | Purpose |
|---|---|---|
| `01-loop-closer-cpp.patch` | `modules/pgo/src/loop_closer.cpp` | Synchronous mode: runs keyframe integration, loop detection, ICP, and optimization inline without worker threads. |
| `02-loop-closer-h.patch` | `modules/pgo/include/bievr_pgo/loop_closer.h` | Adds `Config::synchronous` flag. |
