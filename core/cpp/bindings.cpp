// pybind11 bridge for BIEVR-LIO.
//
// Upstream's estimator is already ROS-free and synchronous: Synchronizer::addImu
// and addPointcloud run Pipeline::processFrame inline as soon as IMU covers a
// sweep end. The bridge therefore mirrors upstream's process_bag.cpp: it loads
// upstream's YAML with upstream's loader, feeds the Synchronizer, and replaces
// the ROS publisher with callbacks registered through
// Pipeline::registerPublisher. No worker thread, no queues: every push returns
// after all processing it triggered has finished.

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tbb/global_control.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "bievr_lio/config_loader.h"
#include "bievr_lio/synchronizer.h"
#include "bievr_lio/utils.h"

namespace py = pybind11;

namespace {

using Array = py::array_t<double, py::array::c_style | py::array::forcecast>;

uint64_t secondsToNs(double s) {
  if (!std::isfinite(s) || s <= 0.0) {
    throw std::invalid_argument("stamp must be a finite, positive time in seconds");
  }
  return static_cast<uint64_t>(std::llround(s * 1e9));
}

bievr::V3 toV3(const Array& a, const char* name) {
  if (a.size() != 3) throw std::invalid_argument(std::string(name) + " must have 3 elements");
  const double* p = a.data();
  return bievr::V3(p[0], p[1], p[2]);
}

class BievrOdometry {
 public:
  BievrOdometry(const std::vector<std::string>& config_files, size_t map_point_stride)
      : map_point_stride_(map_point_stride) {
    if (!bievr::loadConfigFromYaml(config_files, config_)) {
      throw std::invalid_argument("BIEVR-LIO rejected the configuration (see log above)");
    }
    // Terminal UI only; it redraws the screen on every frame.
    config_.pipeline_config.print_dashboard = false;
    // As upstream's process_bag.cpp: cap TBB parallelism for this session.
    if (config_.max_num_threads > 0) {
      tbb_control_ = std::make_unique<tbb::global_control>(
          tbb::global_control::max_allowed_parallelism, config_.max_num_threads);
    }

    pipeline_ = std::make_shared<bievr::Pipeline>(config_.pipeline_config);
    pipeline_->registerPublisher<bievr::Odometry>(
        [this](const bievr::Odometry& odom, const bievr::Header& header, const std::string&,
               const std::string&) { onOdometry(odom, header); });
    pipeline_->registerPublisher<bievr::IntensityPointcloud>(
        [this](const bievr::IntensityPointcloud& cloud, const bievr::Header&,
               const std::string& topic, const std::string&) { onCloud(cloud, topic); });
    pipeline_->registerPublisher<bievr::V3>(
        [this](const bievr::V3& v, const bievr::Header&, const std::string& topic,
               const std::string&) {
          if (topic == "bias/acc") acc_bias_ = v;
          if (topic == "bias/gyro") gyro_bias_ = v;
        });
    // Debug clouds (debug.publish_all_clouds); nothing to export.
    pipeline_->registerPublisher<bievr::Pointcloud>(
        [](const bievr::Pointcloud&, const bievr::Header&, const std::string&,
           const std::string&) {});
    synchronizer_ = std::make_unique<bievr::Synchronizer>(pipeline_);
  }

  bool pushImu(double stamp, const Array& acc, const Array& gyro) {
    bievr::ImuMeasurement imu;
    imu.stamp = secondsToNs(stamp);
    imu.acc = toV3(acc, "acceleration");
    imu.gyro = toV3(gyro, "angular_velocity");
    py::gil_scoped_release release;  // before locking: reacquired after unlock
    std::lock_guard<std::mutex> lock(mutex_);
    const bool accepted = synchronizer_->addImu(imu);
    accepted ? ++imu_accepted_ : ++imu_rejected_;
    return accepted;
  }

  bool pushLidar(double stamp, const Array& points, const Array& times) {
    if (points.ndim() != 2 || (points.shape(1) != 3 && points.shape(1) != 4)) {
      throw std::invalid_argument("points must be (N, 3) or (N, 4) [x, y, z(, intensity)]");
    }
    const size_t n = static_cast<size_t>(points.shape(0));
    const size_t cols = static_cast<size_t>(points.shape(1));
    if (times.ndim() != 1 || static_cast<size_t>(times.shape(0)) != n) {
      throw std::invalid_argument("relative_times must be (N,) matching points");
    }

    // Same layout and stamps as upstream's msgToPointcloud: per-point times are
    // seconds relative to `stamp`, end_stamp is the latest point.
    bievr::StampedIntensityPointcloud cloud;
    cloud.stamp = secondsToNs(stamp);
    cloud.resize(n);
    const double* p = points.data();
    const double* t = times.data();
    double max_time = 0.0;
    for (size_t i = 0; i < n; ++i) {
      if (!std::isfinite(t[i]) || t[i] < 0.0) {
        throw std::invalid_argument("relative_times must be finite and >= 0");
      }
      auto col = cloud[i];
      col << p[i * cols], p[i * cols + 1], p[i * cols + 2], t[i],
          cols == 4 ? p[i * cols + 3] : 0.0;
      max_time = std::max(max_time, t[i]);
    }
    cloud.end_stamp = cloud.stamp + bievr::sToNs(max_time);

    py::gil_scoped_release release;  // before locking: reacquired after unlock
    std::lock_guard<std::mutex> lock(mutex_);
    if (n == 0) {  // upstream's converter drops empty clouds
      ++lidar_empty_;
      return false;
    }
    const bool accepted = synchronizer_->addPointcloud(cloud);
    accepted ? ++lidar_accepted_ : ++lidar_rejected_;
    return accepted;
  }

  py::object latestPose() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (trajectory_.empty()) return py::none();
    py::array_t<double> out(8);
    std::copy(trajectory_.end() - 8, trajectory_.end(), out.mutable_data());
    return out;
  }

  py::array_t<double> trajectory() {
    std::lock_guard<std::mutex> lock(mutex_);
    const py::ssize_t rows = static_cast<py::ssize_t>(trajectory_.size() / 8);
    py::array_t<double> out({rows, py::ssize_t{8}});
    std::copy(trajectory_.begin(), trajectory_.end(), out.mutable_data());
    return out;
  }

  py::array_t<float> drainMapPoints() {
    std::lock_guard<std::mutex> lock(mutex_);
    const py::ssize_t rows = static_cast<py::ssize_t>(map_points_.size() / 4);
    py::array_t<float> out({rows, py::ssize_t{4}});
    std::copy(map_points_.begin(), map_points_.end(), out.mutable_data());
    map_points_.clear();
    map_points_.shrink_to_fit();
    return out;
  }

  py::dict status() {
    std::lock_guard<std::mutex> lock(mutex_);
    py::dict d;
    d["imu_accepted"] = imu_accepted_;
    d["imu_rejected"] = imu_rejected_;
    d["lidar_accepted"] = lidar_accepted_;
    d["lidar_rejected"] = lidar_rejected_;
    d["lidar_empty"] = lidar_empty_;
    d["poses"] = trajectory_.size() / 8;
    d["acc_bias"] = std::vector<double>(acc_bias_.data(), acc_bias_.data() + 3);
    d["gyro_bias"] = std::vector<double>(gyro_bias_.data(), gyro_bias_.data() + 3);
    return d;
  }

 private:
  // Called on the pushing thread from inside processFrame (mutex_ held).
  void onOdometry(const bievr::Odometry& odom, const bievr::Header& header) {
    const bievr::Quaternion q = odom.pose.quaternion();
    const bievr::V3& t = odom.pose.translation();
    trajectory_.insert(trajectory_.end(), {bievr::nsToS(header.stamp), t.x(), t.y(), t.z(),
                                           q.x(), q.y(), q.z(), q.w()});
  }

  void onCloud(const bievr::IntensityPointcloud& cloud, const std::string& topic) {
    if (map_point_stride_ == 0 || topic != "points/registered") return;
    for (size_t i = 0; i < cloud.size(); i += map_point_stride_) {
      const auto pt = cloud[i];
      map_points_.insert(map_points_.end(), {static_cast<float>(pt(0)), static_cast<float>(pt(1)),
                                             static_cast<float>(pt(2)),
                                             static_cast<float>(pt(3))});
    }
  }

  bievr::Config config_;
  size_t map_point_stride_;
  std::unique_ptr<tbb::global_control> tbb_control_;
  std::shared_ptr<bievr::Pipeline> pipeline_;
  std::unique_ptr<bievr::Synchronizer> synchronizer_;
  std::mutex mutex_;

  std::vector<double> trajectory_;  // rows of t, x, y, z, qx, qy, qz, qw
  std::vector<float> map_points_;   // rows of x, y, z, intensity
  bievr::V3 acc_bias_ = bievr::V3::Zero();
  bievr::V3 gyro_bias_ = bievr::V3::Zero();
  size_t imu_accepted_ = 0, imu_rejected_ = 0;
  size_t lidar_accepted_ = 0, lidar_rejected_ = 0, lidar_empty_ = 0;
};

}  // namespace

PYBIND11_MODULE(_core, m) {
  m.doc() = "BIEVR-LIO offline bridge";
  py::class_<BievrOdometry>(m, "BievrOdometry")
      .def(py::init<const std::vector<std::string>&, size_t>(), py::arg("config_files"),
           py::arg("map_point_stride") = 0)
      .def("push_imu", &BievrOdometry::pushImu, py::arg("stamp"), py::arg("acceleration"),
           py::arg("angular_velocity"))
      .def("push_lidar", &BievrOdometry::pushLidar, py::arg("stamp"), py::arg("points"),
           py::arg("relative_times"))
      .def("latest_pose", &BievrOdometry::latestPose)
      .def("trajectory", &BievrOdometry::trajectory)
      .def("drain_map_points", &BievrOdometry::drainMapPoints)
      .def("status", &BievrOdometry::status);
}
