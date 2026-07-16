#pragma once

#include <algorithm>
#include "tools.hpp"
#include "ekf_imu.hpp"
#include "voxel_map.hpp"
#include "feature_point.hpp"
#include "loop_refine.hpp"
#include <mutex>
#include <Eigen/Eigenvalues>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#ifdef VOXELSLAM_WITH_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif
#include <traversability_msgs/msg/key_frame.hpp>
#include <traversability_msgs/msg/key_frame_additions.hpp>
#include <traversability_msgs/msg/key_frame_updates.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <malloc.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include "BTC.h"

using namespace std;

using PubCloud = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;
rclcpp::Node::SharedPtr g_node;
std::shared_ptr<tf2_ros::TransformBroadcaster> g_tf_br;
PubCloud pub_scan, pub_cmap, pub_init, pub_pmap;
PubCloud pub_test, pub_prev_path, pub_curr_path;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
// Corrected (SLAM/map-frame) odometry — jumps at loop closures. Published
// alongside the continuous /Odometry so consumers can pick their world.
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_corrected;
// Keyframe trajectory as stamped poses WITH orientation (nav_msgs/Path),
// republished corrected after loop closure. Consumers (e.g. the keyframe
// terrain map) anchor data to these poses and re-render when they move.
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_kf_path;
// Observation outputs for the REP-105 (continuous-odometry) experiment.
// pub_path_corrected — live trajectory in the SLAM (corrected) world;
//   loop_update() rigidly moves the WHOLE stored history by dx, so this
//   path snaps at a loop closure exactly like the internal map does.
// pub_path_continuous — same trajectory with every closure correction
//   folded into g_T_map_odom instead of the pose; never jumps. This is
//   what a future odom->base_link output would look like.
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_corrected;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_continuous;
// Per-scan pose-health flag for downstream mapping consumers.
// stamp = scan time; frame_id = "degenerate" while scan matching is failing
// (degrade_cnt > 0) or within General.degen_grace seconds after a system
// reset (post-reset settling); "ok" otherwise. The elevation mapping node
// drops point-cloud frames whose stamp falls in a degenerate interval.
rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_degen;
// Keyframe export for the traversability_mapping library (suchetanrs):
// additions when a keyframe settles, updates with corrected poses after a
// loop closure. Created only when General/pub_trav_keyframes is true.
rclcpp::Publisher<traversability_msgs::msg::KeyFrameAdditions>::SharedPtr pub_trav_add;
rclcpp::Publisher<traversability_msgs::msg::KeyFrameUpdates>::SharedPtr pub_trav_upd;
rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl;
#ifdef VOXELSLAM_WITH_LIVOX
rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox;
#endif

// Frame names for the published TF and clouds. Defaults keep the original
// standalone-SLAM behaviour (camera_init/aft_mapped). Override via params
// (e.g. odom/base_link) to plug into a nav2/tf tree. g_odom_topic != "" enables
// a nav_msgs/Odometry publisher.
std::string g_world_frame = "camera_init";
std::string g_body_frame  = "aft_mapped";
std::string g_odom_topic  = "";
// REP-105 split (opt-in): when non-empty, pub_odom_func publishes
//   world_frame -> odom_frame  = g_T_map_odom   (jumps at loop closures)
//   odom_frame  -> body_frame  = continuous pose (never jumps)
// and /Odometry carries the CONTINUOUS pose in odom_frame. When empty
// (default), behaviour is byte-identical to the original single-TF output.
std::string g_odom_frame  = "";
// Traversability keyframe export (off by default; see pub_trav_add above).
// trav_kf_dist throttles additions by travelled distance — the library keeps
// one point cloud in RAM per added keyframe.
bool g_pub_trav = false;
double g_trav_kf_dist = 0.5;
// Accumulated loop-closure correction (the future map->odom transform in
// REP-105 terms). Updated only inside loop_update(), which runs on the same
// odometry thread that publishes — no locking needed. continuous_pose =
// g_T_map_odom.inverse() * corrected_pose is bitwise-continuous across a
// closure because the same dx enters both.
Eigen::Isometry3d g_T_map_odom = Eigen::Isometry3d::Identity();
// Pose-health flag state (see pub_degen). g_last_reset_time is the scan time
// of the last system_reset(); flags stay "degenerate" for g_degen_grace
// seconds afterwards to cover post-reset re-initialization settling.
double g_degen_grace = 3.0;
double g_last_reset_time = -1e18;
// After a system_reset the odom world restarts at the origin: everything
// downstream consumers accumulated in the old odom frame is invalid. The
// first per-scan flag after a reset carries frame_id "reset" (consumers
// treat it as degenerate AND discard their odom-frame state).
bool g_reset_pending = false;
// PV-LIO-style stationary initialization (General.static_init): when the
// first IMU window is stationary, initialize gravity/bias from the IMU
// average and skip the motion BA (whose free-gravity estimate is
// unobservable without excitation and trips the |g| gate -> reset-retry).
bool g_static_init = false;
// Backing buffers for the two observation paths (~2 Hz appends).
nav_msgs::msg::Path g_path_corrected, g_path_continuous;
int g_path_decim = 0;

template <typename T>
void pub_pl_func(T &pl, PubCloud &pub)
{
  // Guard against the shutdown window: worker threads may still call this
  // after the context is invalidated by a SIGINT/SIGTERM handler.
  if(!rclcpp::ok() || !g_node || !pub) return;
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = g_world_frame;
  output.header.stamp = g_node->now();
  pub->publish(output);
}

// Publish the keyframe trajectory as nav_msgs/Path (stamped poses with
// orientation). `tail` may add not-yet-settled keyframes after the main list.
// Pose stamps carry the scan time (x.t) so consumers can match data captured
// at that moment; after loop_update the same stamps reappear with corrected
// poses, signalling a re-anchor.
inline void pub_kf_path_func(const std::vector<ScanPose*> &poses,
                             const std::deque<ScanPose*> &tail)
{
  if(!rclcpp::ok() || !g_node || !pub_kf_path) return;
  nav_msgs::msg::Path path;
  path.header.frame_id = g_world_frame;
  path.header.stamp = g_node->now();
  path.poses.reserve(poses.size() + tail.size());
  auto add = [&path](const IMUST &x)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = path.header.frame_id;
    ps.header.stamp = rclcpp::Time(static_cast<int64_t>(x.t * 1e9));
    ps.pose.position.x = x.p[0];
    ps.pose.position.y = x.p[1];
    ps.pose.position.z = x.p[2];
    Eigen::Quaterniond q(x.R);
    q.normalize();
    ps.pose.orientation.w = q.w();
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    path.poses.push_back(ps);
  };
  for(const ScanPose *bl: poses) add(bl->x);
  for(const ScanPose *bl: tail) add(bl->x);
  pub_kf_path->publish(path);
}

// Fill one traversability_msgs/KeyFrame from a SLAM state. kf_pointcloud is
// left empty on purpose: traversability_node buffers the lidar topic itself
// (use_lidar_pointcloud: true) and matches by timestamp, so the stamp must be
// the scan time x.t (same time base as the lidar message headers).
inline traversability_msgs::msg::KeyFrame make_trav_kf(uint64_t id, const IMUST &x)
{
  traversability_msgs::msg::KeyFrame kf;
  kf.kf_timestamp_in_nanosec = static_cast<uint64_t>(x.t * 1e9);
  kf.kf_id = id;
  kf.map_id = 0;
  kf.kf_pose.position.x = x.p[0];
  kf.kf_pose.position.y = x.p[1];
  kf.kf_pose.position.z = x.p[2];
  Eigen::Quaterniond q(x.R);
  q.normalize();
  kf.kf_pose.orientation.w = q.w();
  kf.kf_pose.orientation.x = q.x();
  kf.kf_pose.orientation.y = q.y();
  kf.kf_pose.orientation.z = q.z();
  return kf;
}

// ROS2 replacement for ros::NodeHandle::param<T>(name, val, default).
// Nested YAML keys are addressed with '.' in ROS2, so "General/lid_topic"
// is translated to "General.lid_topic" before declaring/getting.
template<typename T>
void get_param(const rclcpp::Node::SharedPtr &n, const std::string &name, T &val, const T &def)
{
  std::string key = name;
  std::replace(key.begin(), key.end(), '/', '.');
  if(!n->has_parameter(key))
    n->declare_parameter<T>(key, def);
  n->get_parameter(key, val);
}

mutex mBuf;
Features feat;
deque<sensor_msgs::msg::Imu::SharedPtr> imu_buf;
deque<pcl::PointCloud<PointType>::Ptr> pcl_buf;
deque<double> time_buf;

double imu_last_time = -1;
int point_notime = 0;
double last_pcl_time = -1;

// Constant-velocity fallback for IMU dropouts (Odometry/imu_cv_fallback).
// Holes > g_imu_gap_thresh in the stream are padded with synthetic samples
// (frame_id "cv_synth") that encode zero rotation and zero world acceleration,
// so EKF propagation, deskew and preintegration run unchanged through the gap.
bool g_imu_cv_enable = false;
double g_imu_gap_thresh = 0.1;     // [s] hole size that triggers padding
double g_imu_period_est = 0.01;    // [s] nominal IMU period, EMA of live stream

void imu_handler(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in)
{
  static int flag = 1;
  if(flag)
  {
    flag = 0;
    printf("Time0: %lf\n", toSec(msg_in->header.stamp));
  }

  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

  // For Hilti 2022 exp03
  // double t0 = 1646320760 + 255.5;
  // double t1 = 1646320760 + 256.2;
  // double tc = toSec(msg->header.stamp);
  // if(tc > t0 && tc < t1)
  //   msg->linear_acceleration.z = -9.7;

  mBuf.lock();
  double tc = toSec(msg->header.stamp);
  // Nominal-rate estimate for gap padding; holes themselves are excluded.
  double dt = tc - imu_last_time;
  if(imu_last_time > 0 && dt > 0 && dt < g_imu_gap_thresh)
    g_imu_period_est += 0.05 * (dt - g_imu_period_est);
  imu_last_time = tc;
  imu_buf.push_back(msg);
  mBuf.unlock();
}

// Pad IMU dropouts inside the current scan span with constant-velocity
// pseudo-measurements: raw gyro = bg (bias-corrected rate 0, attitude held),
// raw accel = (ba - R^T g)/scale (bias-corrected specific force cancels
// gravity, so v stays constant and p integrates linearly). Samples are tagged
// "cv_synth" so motion_blur() can inflate their process noise. Returns the
// number of samples inserted.
int fill_imu_gaps(deque<sensor_msgs::msg::Imu::SharedPtr> &imus, IMUST &xc, IMUEKF &ekf)
{
  // Before EKF init there is no velocity/gravity estimate to hold constant.
  if(!g_imu_cv_enable || !ekf.init_flag) return 0;

  Eigen::Vector3d raw_acc = (xc.ba - xc.R.transpose() * xc.g) / ekf.scale_gravity;
  auto make_synth = [&](double t)
  {
    sensor_msgs::msg::Imu::SharedPtr s(new sensor_msgs::msg::Imu());
    fromSec(s->header.stamp, t);
    s->header.frame_id = "cv_synth";
    s->angular_velocity.x = xc.bg[0];
    s->angular_velocity.y = xc.bg[1];
    s->angular_velocity.z = xc.bg[2];
    s->linear_acceleration.x = raw_acc[0];
    s->linear_acceleration.y = raw_acc[1];
    s->linear_acceleration.z = raw_acc[2];
    return s;
  };

  // Hole boundaries: previous scan end -> samples -> current scan end.
  vector<double> ts;
  ts.push_back(ekf.last_pcl_end_time);
  for(auto &imu: imus) ts.push_back(toSec(imu->header.stamp));
  ts.push_back(ekf.pcl_end_time);

  int inserted = 0;
  deque<sensor_msgs::msg::Imu::SharedPtr> filled;
  size_t next_real = 0;
  for(size_t i=0; i+1<ts.size(); i++)
  {
    if(i > 0 && next_real < imus.size())
      filled.push_back(imus[next_real++]);
    double hole = ts[i+1] - ts[i];
    if(hole > g_imu_gap_thresh)
      for(double t = ts[i] + g_imu_period_est; t < ts[i+1] - 0.5*g_imu_period_est; t += g_imu_period_est)
      {
        filled.push_back(make_synth(t));
        inserted++;
      }
  }
  if(inserted > 0)
    imus.swap(filled);
  return inserted;
}

template<class T>
void pcl_handler(T &msg)
{
  pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>());
  double t0 = feat.process(msg, *pl_ptr);

  if(pl_ptr->empty())
  {
    PointType ap; 
    ap.x = 0; ap.y = 0; ap.z = 0; 
    ap.intensity = 0; ap.curvature = 0;
    pl_ptr->push_back(ap);
    ap.curvature = 0.09;
    pl_ptr->push_back(ap);
  }

  sort(pl_ptr->begin(), pl_ptr->end(), [](PointType &x, PointType &y)
  {
    return x.curvature < y.curvature;
  });
  while(pl_ptr->back().curvature > 0.11)
    pl_ptr->points.pop_back();

  mBuf.lock();
  time_buf.push_back(t0);
  pcl_buf.push_back(pl_ptr);
  mBuf.unlock();
}

bool sync_packages(pcl::PointCloud<PointType>::Ptr &pl_ptr, deque<sensor_msgs::msg::Imu::SharedPtr> &imus, IMUEKF &p_imu)
{
  static bool pl_ready = false;

  if(!pl_ready)
  {
    if(pcl_buf.empty()) return false;

    mBuf.lock();
    pl_ptr = pcl_buf.front();
    p_imu.pcl_beg_time = time_buf.front();
    pcl_buf.pop_front(); time_buf.pop_front();
    mBuf.unlock();

    p_imu.pcl_end_time = p_imu.pcl_beg_time + pl_ptr->back().curvature;

    if(point_notime)
    {
      if(last_pcl_time < 0)
      {
        last_pcl_time = p_imu.pcl_beg_time;
        return false;
      }

      p_imu.pcl_end_time = p_imu.pcl_beg_time;
      p_imu.pcl_beg_time = last_pcl_time;
      last_pcl_time = p_imu.pcl_end_time;
    }

    pl_ready = true;
  }

  if(!pl_ready || imu_last_time <= p_imu.pcl_end_time) return false;

  mBuf.lock();
  double imu_time = toSec(imu_buf.front()->header.stamp);
  while((!imu_buf.empty()) && (imu_time < p_imu.pcl_end_time))
  {
    imu_time = toSec(imu_buf.front()->header.stamp);
    if(imu_time > p_imu.pcl_end_time) break;
    imus.push_back(imu_buf.front());
    imu_buf.pop_front();
  }
  mBuf.unlock();

  if(imu_buf.empty())
  {
    printf("imu buf empty\n"); exit(0);
  }

  pl_ready = false;

  if(imus.size() > 4)
    return true;
  // Under-covered scan: normally dropped, but with the constant-velocity
  // fallback active fill_imu_gaps() pads the hole downstream.
  if(g_imu_cv_enable && p_imu.init_flag)
    return true;
  return false;
}

double dept_err, beam_err;
void calcBodyVar(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &var) 
{
  if (pb[2] == 0)
    pb[2] = 0.0001;
  float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
  float range_var = range_inc * range_inc;
  Eigen::Matrix2d direction_var;
  direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);
  Eigen::Vector3d direction(pb);
  direction.normalize();
  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;
  Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);
  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
  var = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
};

// Compute the variance of the each point
void var_init(IMUST &ext, pcl::PointCloud<PointType> &pl_cur, PVecPtr pptr, double dept_err, double beam_err)
{
  int plsize = pl_cur.size();
  pptr->clear();
  pptr->resize(plsize);
  for(int i=0; i<plsize; i++)
  {
    PointType &ap = pl_cur[i];
    pointVar &pv = pptr->at(i);
    pv.pnt << ap.x, ap.y, ap.z;
    pv.intensity = ap.intensity;
    calcBodyVar(pv.pnt, dept_err, beam_err, pv.var);
    pv.pnt = ext.R * pv.pnt + ext.p;
    pv.var = ext.R * pv.var * ext.R.transpose();
  }
}

void pvec_update(PVecPtr pptr, IMUST &x_curr, PLV(3) &pwld)
{
  Eigen::Matrix3d rot_var = x_curr.cov.block<3, 3>(0, 0);
  Eigen::Matrix3d tsl_var = x_curr.cov.block<3, 3>(3, 3);

  for(pointVar &pv: *pptr)
  {
    Eigen::Matrix3d phat = hat(pv.pnt);
    pv.var = x_curr.R * pv.var * x_curr.R.transpose() + phat * rot_var * phat.transpose() + tsl_var;
    pwld.push_back(x_curr.R * pv.pnt + x_curr.p);
  }
}

// Read the alidarstate.txt
void read_lidarstate(string filename, vector<ScanPose*> &bl_tem)
{
  ifstream file(filename);
  if(!file.is_open())
  {
    printf("Error: %s not found\n", filename.c_str());
    exit(0);
  }

  string lineStr, str;
  vector<double> nums;
  while(getline(file, lineStr))
  {
    nums.clear();
    stringstream ss(lineStr);
    while(getline(ss, str, ' '))
      nums.push_back(stod(str));
    
    IMUST xx;
    xx.t = nums[0];
    xx.p << nums[1], nums[2], nums[3];
    xx.R = Eigen::Quaterniond(nums[7], nums[4], nums[5], nums[6]).matrix();

    if(nums.size() >= 20)
    {
      xx.v << nums[8], nums[9], nums[10];
      xx.bg << nums[11], nums[12], nums[13];
      xx.ba << nums[14], nums[15], nums[16];
      xx.g << nums[17], nums[18], nums[19];
    }

    ScanPose* blp = new ScanPose(xx, nullptr);
    bl_tem.push_back(blp);

    if(nums.size() >= 26)
      for(int i=0; i<6; i++) 
        blp->v6[i] = nums[i + 20];
  }
}

double get_memory()
{
  ifstream infile("/proc/self/status");
  double mem = -1;
  string lineStr, str;
  while(getline(infile, lineStr))
  {
    stringstream ss(lineStr);
    bool is_find = false;
    while(ss >> str)
    {
      if(str == "VmRSS:")
      {
        is_find = true; continue;
      }

      if(is_find) mem = stod(str);
      break;
    }
    if(is_find) break;
  }
  return mem / (1048576);
}

void icp_check(pcl::PointCloud<PointType> &pl_src, pcl::PointCloud<PointType> &pl_tar, PubCloud &pub_src, PubCloud &pub_tar, pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform, IMUST &xx)
{
  pcl::PointCloud<PointType> pl1, pl2;
  for(PointType ap: pl_src.points)
  {
    Eigen::Vector3d v(ap.x, ap.y, ap.z);
    v = loop_transform.second * v + loop_transform.first;
    v = xx.R * v + xx.p;
    ap.x = v[0]; ap.y = v[1]; ap.z = v[2];
    pl1.push_back(ap);
  }
  for(PointType ap: pl_tar.points)
  {
    Eigen::Vector3d v(ap.x, ap.y, ap.z);
    v = xx.R * v + xx.p;
    ap.x = v[0]; ap.y = v[1]; ap.z = v[2];
    pl2.push_back(ap);
  }
  pub_pl_func(pl1, pub_src); pub_pl_func(pl2, pub_tar);
}

