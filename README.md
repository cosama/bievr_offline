# BIEVR Offline

ROS-free Python package and pybind11 bridge for [BIEVR-LIO](https://github.com/ethz-asl/BIEVR-LIO).

## Layout

- `core/`: importable `bievr_lio` package and native C++ extension.
  - `CMakeLists.txt`: stages upstream sources and compiles `cpp/bindings.cpp` into `bievr_lio._core`.
  - `cpp/bindings.cpp`: synchronous bridge replacing upstream ROS interfaces.
  - `python/bievr_lio/`: `BievrOdometry` wrapper class.
  - `tests/test_bievr_lio.py`: API contract and sensor configuration tests.
- `docker/`: container build for running the bridge in isolated environments.
