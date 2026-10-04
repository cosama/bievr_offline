"""Offline BIEVR-LIO session.

Configuration is upstream's own YAML (``config/params.yaml`` plus a sensor
config), loaded by upstream's ``loadConfigFromYaml``: later files override
earlier ones per key, exactly as ``--params_file`` / ``--sensor_config_file``.
The live dashboard (``debug.dashboard``) is forced off.

Every call is synchronous. Upstream's synchronizer processes a sweep inline as
soon as IMU covers its last point, so when ``push_imu``/``push_lidar`` return,
all processing they triggered is done. Results depend only on the pushed data.

* IMU: seconds, m/s^2 (or g; upstream autodetects), rad/s, in the IMU frame.
* LiDAR: points in the LiDAR frame; ``calibration`` maps LiDAR -> IMU.
* Poses are the IMU frame in upstream's world frame, stamped at sweep end.
* As upstream, the first ``imu.t_init`` seconds estimate biases assuming the
  sensor is static, and a sweep still waiting for IMU at the end of the input
  is never processed.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from ._core import BievrOdometry as _NativeBievrOdometry

__all__ = ["BievrOdometry"]

PathLike = str | os.PathLike


class BievrOdometry:
    """One BIEVR-LIO session."""

    def __init__(
        self,
        config_files: PathLike | Sequence[PathLike],
        *,
        map_point_stride: int = 0,
    ):
        """``config_files``: one or more upstream YAML files, later ones win.

        ``map_point_stride > 0`` exports every N-th point of each registered
        sweep (see ``drain_map_points``).
        """
        if isinstance(config_files, (str, os.PathLike)):
            config_files = [config_files]
        self.config_files = [Path(p) for p in config_files]
        for path in self.config_files:
            if not path.is_file():
                raise FileNotFoundError(path)
        if map_point_stride < 0:
            raise ValueError("map_point_stride must be >= 0")
        self._native = _NativeBievrOdometry(
            [str(p) for p in self.config_files], int(map_point_stride)
        )

    def push_imu(self, stamp: float, acceleration, angular_velocity) -> bool:
        """Push one IMU sample. False if upstream rejected it (out of order)."""
        return self._native.push_imu(float(stamp), acceleration, angular_velocity)

    def push_lidar(
        self, stamp: float, points: np.ndarray, relative_times: np.ndarray
    ) -> bool:
        """Push one sweep. False if dropped (empty, or out of order).

        ``stamp`` is the time of the earliest point, ``points`` is (N, 3) or
        (N, 4) XYZ[I], ``relative_times`` (N,) are offsets from ``stamp`` in s.
        """
        return self._native.push_lidar(float(stamp), points, relative_times)

    def latest_pose(self) -> np.ndarray | None:
        """Latest pose [t, x, y, z, qx, qy, qz, qw], or None before the first."""
        return self._native.latest_pose()

    def trajectory(self) -> np.ndarray:
        """All poses so far, (N, 8): t, x, y, z, qx, qy, qz, qw."""
        return self._native.trajectory()

    def drain_map_points(self) -> np.ndarray:
        """World-frame points (M, 4) XYZI registered since the last call."""
        return self._native.drain_map_points()

    def status(self) -> dict[str, Any]:
        """Counters (accepted/rejected inputs, poses) and the current biases."""
        return dict(self._native.status())
