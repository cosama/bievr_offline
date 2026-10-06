# BIEVR Offline

ROS-free Python package and pybind11 bridge for [BIEVR-LIO](https://github.com/ethz-asl/BIEVR-LIO):
the odometry plus its Scan Context / GTSAM loop closure (`modules/`), run synchronously.

## Layout

- `core/`: importable `bievr` package and native C++ extension.
  - `CMakeLists.txt`: stages upstream sources and compiles `cpp/bindings.cpp` into `bievr._core`.
  - `cpp/bindings.cpp`: synchronous bridge replacing upstream ROS interfaces.
  - `python/bievr/`: `Bievr` wrapper class.
  - `tests/test_bievr.py`: API contract and sensor configuration tests.
- `docker/`: container build for running the bridge in isolated environments.
