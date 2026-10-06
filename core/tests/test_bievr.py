"""API contract tests for the BIEVR offline bridge.

Estimation quality and determinism on real data: see
BIEVR_LIO_INTEGRATION_STATUS.md (50CPatio, bit-identical across thread counts).
"""

from pathlib import Path

import numpy as np
import pytest

from bievr import Bievr

UPSTREAM_CONFIG = Path(__file__).resolve().parents[4] / "upstream/BIEVR-LIO/config"
PARAMS = UPSTREAM_CONFIG / "params.yaml"
SENSORS = sorted((UPSTREAM_CONFIG / "sensor_configs").glob("*.yaml"))


def box_sweep(rng, n=6000, half=(6.0, 4.0, 2.5)):
    """Points on the walls of a box around the origin."""
    pts = rng.uniform(-1.0, 1.0, size=(n, 3)) * half
    axis = rng.integers(0, 3, n)
    side = rng.choice([-1.0, 1.0], n)
    pts[np.arange(n), axis] = side * np.asarray(half)[axis]
    return np.column_stack((pts, rng.uniform(0, 100, n)))


def run_static(config, seconds=3.0, stride=0):
    """Static sensor in a box: 200 Hz IMU, 10 Hz sweeps, interleaved like a bag."""
    session = Bievr(config, map_point_stride=stride)
    rng = np.random.default_rng(0)
    t0 = 1_700_000_000.0
    next_sweep = t0
    for t in t0 + np.arange(0.0, seconds, 0.005):
        session.push_imu(t, [0.0, 0.0, 9.81], [0.0, 0.0, 0.0])
        if t >= next_sweep + 0.1:  # a sweep arrives after its last point
            pts = box_sweep(rng)
            session.push_lidar(next_sweep, pts, np.linspace(0.0, 0.099, len(pts)))
            next_sweep += 0.1
    return session


@pytest.fixture
def config(tmp_path):
    path = tmp_path / "bievr.yaml"
    path.write_text(
        PARAMS.read_text()
        + "\ncalibration:\n  translation: [0, 0, 0]\n"
        "  rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n"
    )
    return path


@pytest.mark.parametrize("sensor", SENSORS, ids=lambda p: p.stem)
def test_upstream_configs_load_unchanged(sensor):
    Bievr([PARAMS, sensor])


def test_invalid_config_raises(tmp_path):
    with pytest.raises(ValueError, match="rejected"):
        Bievr(PARAMS)  # no calibration: upstream refuses
    bad = tmp_path / "bad.yaml"
    bad.write_text(SENSORS[0].read_text() + "\nmap:\n  voxel_size_m: -1\n")
    with pytest.raises(ValueError, match="rejected"):
        Bievr([PARAMS, bad])
    with pytest.raises(FileNotFoundError):
        Bievr(tmp_path / "missing.yaml")


def test_static_scene_tracks_and_is_deterministic(config):
    a = run_static(config, stride=10)
    traj = a.trajectory()
    status = a.status()
    assert traj.shape[1] == 8 and len(traj) >= 25
    assert status["poses"] == len(traj)
    assert status["lidar_rejected"] == status["imu_rejected"] == 0
    assert np.all(np.diff(traj[:, 0]) > 0)
    assert np.abs(traj[:, 1:4]).max() < 0.05
    np.testing.assert_array_equal(a.latest_pose(), traj[-1])
    scans = a.drain_map_scans()
    assert len(scans) > 0
    assert np.isin([stamp for stamp, _ in scans], traj[:, 0]).all()
    for _, points in scans:
        assert points.dtype == np.float32 and points.shape[1] == 4 and len(points) > 0
    assert a.drain_map_scans() == []
    assert a.status()["map_scans_dropped"] == 0

    b = run_static(config, stride=10)
    np.testing.assert_array_equal(b.trajectory(), traj)


def test_push_contract(config):
    session = Bievr(config)
    assert session.latest_pose() is None
    assert session.trajectory().shape == (0, 8)
    assert session.keyframe_trajectory().shape == (0, 8)
    assert session.push_imu(10.0, [0, 0, 9.81], [0, 0, 0])
    assert not session.push_imu(9.0, [0, 0, 9.81], [0, 0, 0])  # out of order
    assert not session.push_lidar(10.0, np.zeros((0, 3)), np.zeros(0))
    status = session.status()
    assert (status["imu_rejected"], status["lidar_empty"]) == (1, 1)
    assert status["loop_closure"]["enabled"] is True
    with pytest.raises(ValueError):
        session.push_lidar(10.0, np.zeros((5, 2)), np.zeros(5))
    with pytest.raises(ValueError):
        session.push_lidar(10.0, np.zeros((5, 3)), np.zeros(4))
    with pytest.raises(ValueError):
        session.push_lidar(10.0, np.zeros((5, 3)), -np.ones(5))
    with pytest.raises(ValueError):
        session.push_imu(float("nan"), [0, 0, 9.81], [0, 0, 0])
    with pytest.raises(ValueError):
        Bievr(config, map_point_stride=-1)


def test_disabled_loop_closure(config, tmp_path):
    disable_pgo = tmp_path / "disable_pgo.yaml"
    disable_pgo.write_text("loop_closure:\n  enable: false\n")
    session = Bievr([config, disable_pgo])
    assert session.status()["loop_closure"]["enabled"] is False
    assert session.keyframe_trajectory().shape == (0, 8)


def test_loop_closure_tracks_and_is_deterministic(tmp_path):
    pgo_cfg = tmp_path / "bievr_pgo.yaml"
    pgo_cfg.write_text(
        PARAMS.read_text()
        + "\ncalibration:\n  translation: [0, 0, 0]\n"
        "  rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n"
        "loop_closure:\n"
        "  enable: true\n"
        "  keyframe_meter_gap: 0.1\n"
        "  keyframe_deg_gap: 1.0\n"
    )
    a = run_static(pgo_cfg, seconds=2.0)
    traj_a = a.trajectory()
    kf_a = a.keyframe_trajectory()
    status_a = a.status()

    assert traj_a.shape[1] == 8 and len(traj_a) >= 15
    assert status_a["loop_closure"]["enabled"] is True
    assert status_a["loop_closure"]["keyframes"] >= 1
    assert status_a["loop_closure"]["dropped_frames"] == 0
    assert kf_a.shape[0] == status_a["loop_closure"]["keyframes"]
    assert np.isin(kf_a[:, 0], traj_a[:, 0]).all()

    b = run_static(pgo_cfg, seconds=2.0)
    np.testing.assert_array_equal(b.trajectory(), traj_a)
    np.testing.assert_array_equal(b.keyframe_trajectory(), kf_a)
