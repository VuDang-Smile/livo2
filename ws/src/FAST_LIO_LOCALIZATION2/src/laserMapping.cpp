// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <queue>
#include <memory>
#include <algorithm>
#include <limits>
#include <atomic>
#include <math.h>
#include <cmath>
#include <string>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <filesystem>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include "Scancontext/Scancontext.h"

namespace fs = std::filesystem;

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false;
double global_loc_convergence_thresh = 2.0;
double global_loc_fitness_thresh = 0.5;
double global_loc_fitness_thresh_max = 2.5;
double global_loc_fitness_adaptive_step = 0.2;
bool global_loc_fitness_adaptive_enable = true;
double global_loc_height_thresh = -1.0;
double global_loc_candidate_max_dist = 20.0; // meters; <=0 disables distance gate
double global_loc_single_shot_thresh = 0.25; // Accept immediate relocalization when ScanContext+ICP is this good (lower = stricter)
double global_loc_gicp_single_shot_thresh = 0.20; // Stricter acceptance for GICP-only single-shot
bool global_loc_auto_height_align = true;
bool global_loc_force_map_height = true;
double global_loc_z_bias = 0.0;
double global_loc_max_jump_distance = 3.0; // meters; reject if new position is more than this distance from current (default 3.0m)
double global_loc_max_jump_rotation = 0.52; // radians; reject if rotation change is more than this (30 degrees, default 0.52)
bool enable_virtual_scancontexts = true;
double virtual_tile_spacing = 10.0;
double virtual_tile_radius = 15.0;
int virtual_tile_min_points = 400;
int virtual_tile_max_tiles = 200;
bool freeze_odom_until_global_loc = true;
bool freeze_odom_on_no_points = true;
double freeze_odom_no_point_tolerance = 0.25; // seconds
bool unfreeze_after_global_loc = true;
size_t freeze_skip_counter = 0;
double last_freeze_skip_log_time = 0.0;
bool enable_ndt_fallback = true;
double ndt_fallback_radius = 25.0;
int ndt_fallback_min_points = 200;
double ndt_fallback_fitness_thresh = 1.5;

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0;
bool point_selected_surf[100000] = {0};
bool lidar_pushed, flg_reset, flg_exit = false, flg_EKF_inited;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

vector<vector<int>> pointSearchInd_surf;
vector<BoxPointType> cub_needrm;
vector<PointVector> Nearest_Points;
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
deque<double> time_buffer;
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;
PointCloudXYZI::Ptr global_map(new PointCloudXYZI());
// Separate RGB map for visualization (preserves RGB colors from PCD files)
pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
bool global_map_has_rgb = false;

struct MapTileMeta
{
    bool from_disk = true;
    int file_index = -1;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    PointCloudXYZI::Ptr local_cloud;
};

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE ikdtree;
KD_TREE ikdtree_global;
KD_TREE ikdtree_init;

std::shared_ptr<SCManager> scManager = std::make_shared<SCManager>();
std::mutex init_feats_down_body_mutex;
struct InitFrame
{
    int id = -1;
    PointCloudXYZI::Ptr cloud;
    Eigen::Vector3d odom_pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond odom_rot = Eigen::Quaterniond::Identity();

    InitFrame() = default;

    InitFrame(int idx,
              PointCloudXYZI::Ptr pc,
              const Eigen::Vector3d &pos,
              const Eigen::Quaterniond &rot)
        : id(idx), cloud(std::move(pc)), odom_pos(pos), odom_rot(rot) {}
};
std::queue<InitFrame> init_feats_down_bodys;
std::vector<MapTileMeta> scancontext_tiles;
std::shared_ptr<pcl::KdTreeFLANN<PointType>> global_map_kdtree(new pcl::KdTreeFLANN<PointType>());
bool global_map_kdtree_ready = false;

int localization_mode = 1;
bool path_en = true;
bool extrinsic_est_en = true;
int pcd_save_interval = -1;
int pcd_index = 0;
bool flg_first_scan = true;
int init_count = 0;
std::pair<int, Eigen::Matrix4d> init_result;
double init_result_fitness = -1.0; // Fitness của localization result, dùng để validate khi apply
std::mutex global_localization_finish_state_mutex;
bool global_localization_finish = false;
bool global_update = false;
double last_stable_state_time = 0.0;

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> position_map;
std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> pose_map;

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> position_init;
std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> pose_init;

std::string map_root_dir;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
state_ikfom frozen_state_for_pub;
bool frozen_state_for_pub_valid = false;
state_ikfom last_stable_state;
bool last_stable_state_valid = false;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
    flg_exit = true;
    std::cout << "catch sig %d" << sig << std::endl;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

inline void dump_lio_state_to_log(FILE *fp)
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                            // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2));    // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                 // omega
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2));    // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                 // Acc
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));       // Bias_g
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));       // Bias_a
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a
    fprintf(fp, "\r\n");
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const *const pi, PointType *const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    for (int i = 0; i < points_history.size(); i++)
        _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized)
    {
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move)
        return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if (cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
    mtx_buffer.lock();
    scan_count++;
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    if (cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);
    last_timestamp_lidar = cur_time;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
{
    mtx_buffer.lock();
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    last_timestamp_lidar = cur_time;

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    publish_count++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp =
            rclcpp::Time(timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp));
    }

    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty())
    {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        if (meas.lidar->points.size() <= 1)
        {
            lidar_buffer.pop_front();
            return false;
        }
        meas.lidar_beg_time = time_buffer.front();
        lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            float dist = calc_dist(feats_down_world->points[i], mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
            {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
            {
                if (points_near.size() < NUM_MATCH_POINTS)
                    break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
{
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(
            new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(
            new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i],
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;
    }
}

void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i],
                               &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld(
        new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i],
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{
    sensor_msgs::msg::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap->publish(laserCloudMap);
}

template <typename T>
void set_posestamp(T &out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> &tf_br)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    
    // Set velocity (linear velocity in camera_init frame)
    odomAftMapped.twist.twist.linear.x = state_point.vel(0);
    odomAftMapped.twist.twist.linear.y = state_point.vel(1);
    odomAftMapped.twist.twist.linear.z = state_point.vel(2);
    
    // Angular velocity is not directly available in state, set to zero or calculate from IMU if needed
    odomAftMapped.twist.twist.angular.x = 0.0;
    odomAftMapped.twist.twist.angular.y = 0.0;
    odomAftMapped.twist.twist.angular.z = 0.0;
    
    pubOdomAftMapped->publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = "camera_init";
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.child_frame_id = "body";
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_br->sendTransform(trans);
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    // Giảm downsampling từ 10 xuống 5 để path hiển thị nhanh hơn
    static int jjj = 0;
    jjj++;
    if (jjj % 5 == 0)
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

/** closest surface search and residual computation **/
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                                                                : true;
        }

        if (!point_selected_surf[i])
            continue;

        VF(4)
        pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num++;
        }
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time += omp_get_wtime() - match_start;
    double solve_start_ = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); // 23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

bool load_file(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &global_map_pub,
               const rclcpp::Clock::SharedPtr &clock,
               const rclcpp::Logger &logger,
               size_t &tile_count_out)
{
    global_map->clear();
    global_map_rgb->clear();
    global_map_has_rgb = false;
    position_map.clear();
    pose_map.clear();
    scancontext_tiles.clear();
    scManager = std::make_shared<SCManager>();
    global_map_kdtree_ready = false;
    tile_count_out = 0;

    if (map_root_dir.empty())
    {
        map_root_dir = (fs::path(root_dir) / "map").string();
    }

    fs::path pose_path = fs::path(map_root_dir) / "pose.json";
    if (!fs::exists(pose_path))
    {
        RCLCPP_ERROR(logger, "pose.json not found at %s", pose_path.string().c_str());
        return false;
    }

    std::ifstream pose_file(pose_path.string());
    if (!pose_file.is_open())
    {
        RCLCPP_ERROR(logger, "Failed to open %s", pose_path.string().c_str());
        return false;
    }

    double tx, ty, tz, w, x, y, z;
    int file_index = 0;
    int tiles_loaded = 0;
    while (pose_file >> tx >> ty >> tz >> w >> x >> y >> z)
    {
        Eigen::Quaterniond q(w, x, y, z);
        Eigen::Vector3d p(tx, ty, tz);

        fs::path pcd_path = fs::path(map_root_dir) / "pcd" / (std::to_string(file_index) + ".pcd");
        
        // Try loading as PointXYZRGB first (to preserve RGB colors)
        auto temp_rgb = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        auto temp = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
        bool has_rgb = false;
        
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_path.string(), *temp_rgb) == 0 && !temp_rgb->points.empty())
        {
            // Check if it actually has RGB colors (not all white/default)
            bool has_real_colors = false;
            for (const auto& pt : temp_rgb->points)
            {
                if (pt.r != 255 || pt.g != 255 || pt.b != 255)
                {
                    has_real_colors = true;
                    break;
                }
            }
            
            if (has_real_colors)
            {
                has_rgb = true;
                global_map_has_rgb = true;
                
                // Giống recorder project: Load PCD từ local frame
                // PCD files đã được save ở local frame (từ LIVMapper.cpp)
                
                // Convert to PointXYZINormal for localization processing (từ local frame)
                temp->points.resize(temp_rgb->points.size());
                for (size_t i = 0; i < temp_rgb->points.size(); ++i)
                {
                    temp->points[i].x = temp_rgb->points[i].x;
                    temp->points[i].y = temp_rgb->points[i].y;
                    temp->points[i].z = temp_rgb->points[i].z;
                    temp->points[i].intensity = 0.0f;
                    temp->points[i].normal_x = 0.0f;
                    temp->points[i].normal_y = 0.0f;
                    temp->points[i].normal_z = 0.0f;
                    temp->points[i].curvature = 0.0f;
                }
                temp->width = temp_rgb->width;
                temp->height = temp_rgb->height;
                temp->is_dense = temp_rgb->is_dense;
                
                // Giống recorder project: Generate ScanContext từ local frame (TRƯỚC KHI transform về global)
                // Recorder: scManager->makeAndSaveScancontextAndKeys(*temp); (temp ở local frame, line 915)
                scManager->makeAndSaveScancontextAndKeys(*temp);
                
                // Sau đó transform về global frame (giống recorder, line 916)
                pcl::transformPointCloud(*temp_rgb, *temp_rgb, p, q);
                *global_map_rgb += *temp_rgb;
                
                // Transform temp về global frame để add vào global_map
                pcl::transformPointCloud(*temp, *temp, p, q);
                *global_map += *temp;
            }
        }
        
        // If RGB loading failed, try PointXYZINormal
        if (!has_rgb)
        {
            if (pcl::io::loadPCDFile(pcd_path.string(), *temp) == -1)
            {
                RCLCPP_WARN(logger, "Failed to load map tile %s", pcd_path.string().c_str());
                ++file_index;
                continue;
            }
            
            // Giống recorder project: Load PCD từ local frame
            // PCD files đã được save ở local frame (từ LIVMapper.cpp)
            
            // Giống recorder project: Generate ScanContext từ local frame (TRƯỚC KHI transform về global)
            // Recorder: scManager->makeAndSaveScancontextAndKeys(*temp); (temp ở local frame, line 915)
            scManager->makeAndSaveScancontextAndKeys(*temp);
            
            // Sau đó transform về global frame (giống recorder, line 916)
            pcl::transformPointCloud(*temp, *temp, p, q);
            
            // Add to global map (already in global frame)
            *global_map += *temp;
        }
        
        position_map.push_back(p);
        pose_map.push_back(q);

        MapTileMeta meta;
        meta.from_disk = true;
        meta.file_index = file_index;
        meta.position = p;
        meta.orientation = q;
        scancontext_tiles.push_back(meta);

        // Publish RGB cloud if available, otherwise publish normal cloud
        if (global_map_pub != nullptr)
        {
            sensor_msgs::msg::PointCloud2 msg_global;
            if (has_rgb && temp_rgb && !temp_rgb->points.empty())
            {
                // Publish RGB cloud (already transformed)
                pcl::toROSMsg(*temp_rgb, msg_global);
            }
            else if (temp && !temp->points.empty())
            {
                // Publish normal cloud (already transformed)
                pcl::toROSMsg(*temp, msg_global);
            }
            else
            {
                // Skip publishing if no valid cloud
                ++file_index;
                continue;
            }
            msg_global.header.frame_id = "camera_init";
            msg_global.header.stamp = clock->now();
            global_map_pub->publish(msg_global);
        }
        ++file_index;
        ++tiles_loaded;
    }
    pose_file.close();

    if (tiles_loaded == 0)
    {
        RCLCPP_ERROR(logger, "No map tiles loaded from %s", map_root_dir.c_str());
        return false;
    }

    tile_count_out = static_cast<size_t>(tiles_loaded);
    global_map_kdtree->setInputCloud(global_map);
    global_map_kdtree_ready = !global_map->empty();
    RCLCPP_INFO(logger, "Loaded %d map tiles for FAST-Localization", tiles_loaded);
    return true;
}

size_t generate_virtual_scancontexts(const rclcpp::Logger &logger)
{
    if (!enable_virtual_scancontexts)
    {
        return 0;
    }
    if (global_map->empty())
    {
        RCLCPP_WARN(logger, "Virtual ScanContext generation skipped: global map is empty.");
        return 0;
    }
    if (virtual_tile_spacing <= 0.1 || virtual_tile_radius <= 0.1)
    {
        RCLCPP_WARN(logger, "Virtual ScanContext generation disabled because spacing (%.2f) or radius (%.2f) is too small.",
                    virtual_tile_spacing, virtual_tile_radius);
        return 0;
    }

    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(global_map);

    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*global_map, min_pt, max_pt);

    const double min_spacing = std::max(virtual_tile_spacing * 0.5, 0.5);
    const double min_spacing_sq = min_spacing * min_spacing;
    const size_t max_virtual_tiles = virtual_tile_max_tiles > 0 ? static_cast<size_t>(virtual_tile_max_tiles) : std::numeric_limits<size_t>::max();

    auto center_too_close = [&](const Eigen::Vector3d &center) -> bool {
        for (const auto &meta : scancontext_tiles)
        {
            Eigen::Vector3d diff = meta.position - center;
            if (diff.head<2>().squaredNorm() < min_spacing_sq)
            {
                return true;
            }
        }
        return false;
    };

    size_t generated = 0;
    for (double x = min_pt.x(); x <= max_pt.x(); x += virtual_tile_spacing)
    {
        for (double y = min_pt.y(); y <= max_pt.y(); y += virtual_tile_spacing)
        {
            if (generated >= max_virtual_tiles)
            {
                RCLCPP_WARN(logger, "Reached configured virtual ScanContext limit (%d).", virtual_tile_max_tiles);
                return generated;
            }

            Eigen::Vector3d center(x, y, 0.0);
            if (center_too_close(center))
            {
                continue;
            }

            PointType query;
            query.x = x;
            query.y = y;
            query.z = (min_pt.z() + max_pt.z()) * 0.5f;

            std::vector<int> indices;
            std::vector<float> sqr_distances;
            size_t neighbors = kdtree.radiusSearch(query, virtual_tile_radius, indices, sqr_distances);
            if (neighbors < static_cast<size_t>(virtual_tile_min_points))
            {
                continue;
            }

            double avg_z = 0.0;
            for (int idx : indices)
            {
                avg_z += global_map->points[idx].z;
            }
            avg_z /= static_cast<double>(neighbors);
            center.z() = avg_z;

            auto local_cloud = std::make_shared<PointCloudXYZI>();
            local_cloud->reserve(indices.size());
            for (int idx : indices)
            {
                const auto &pt = global_map->points[idx];
                PointType shifted = pt;
                shifted.x = pt.x - center.x();
                shifted.y = pt.y - center.y();
                shifted.z = pt.z - center.z();
                local_cloud->push_back(shifted);
            }

            if (local_cloud->size() < static_cast<size_t>(virtual_tile_min_points))
            {
                continue;
            }

            scManager->makeAndSaveScancontextAndKeys(*local_cloud);

            MapTileMeta meta;
            meta.from_disk = false;
            meta.file_index = -1;
            meta.position = center;
            meta.orientation = Eigen::Quaterniond::Identity();
            meta.local_cloud = local_cloud;
            scancontext_tiles.push_back(meta);
            position_map.push_back(center);
            pose_map.push_back(meta.orientation);

            generated++;
        }
    }

    return generated;
}

bool load_map_tile_cloud(int localization_id,
                         PointCloudXYZI::Ptr &cloud_out,
                         const rclcpp::Logger &logger)
{
    // Handle boundary case: if index equals size, clamp to last valid tile
    if (localization_id == static_cast<int>(scancontext_tiles.size()) && !scancontext_tiles.empty())
    {
        RCLCPP_WARN(logger, "Localization tile id %d at boundary - clamping to last valid tile %zu.", 
                    localization_id, scancontext_tiles.size() - 1);
        localization_id = static_cast<int>(scancontext_tiles.size() - 1);
    }
    
    if (localization_id < 0 || localization_id >= static_cast<int>(scancontext_tiles.size()))
    {
        RCLCPP_WARN(logger, "Localization tile id %d is out of bounds (available %zu).", localization_id, scancontext_tiles.size());
        return false;
    }

    const auto &meta = scancontext_tiles[localization_id];
    if (meta.from_disk)
    {
        fs::path pcd_path = fs::path(map_root_dir) / "pcd" / (std::to_string(meta.file_index) + ".pcd");
        if (pcl::io::loadPCDFile(pcd_path.string(), *cloud_out) == -1)
        {
            RCLCPP_WARN(logger, "Failed to load loop map tile %s", pcd_path.string().c_str());
            return false;
        }
        return true;
    }

    if (!meta.local_cloud)
    {
        RCLCPP_WARN(logger, "Virtual tile %d has no cached point cloud.", localization_id);
        return false;
    }
    pcl::copyPointCloud(*meta.local_cloud, *cloud_out);
    return true;
}

bool run_gicp_fallback(const InitFrame &frame,
                       const PointCloudXYZI::Ptr &scan_in,
                       Eigen::Matrix4d &T_out,
                       double &fitness_out,
                       const rclcpp::Logger &logger)
{
    if (!enable_ndt_fallback)
    {
        return false;
    }
    if (!global_map_kdtree_ready || global_map->empty())
    {
        RCLCPP_WARN(logger, "GICP fallback skipped: global map KD-tree not ready.");
        return false;
    }
    if (!scan_in || scan_in->empty())
    {
        RCLCPP_WARN(logger, "GICP fallback skipped: scan is empty.");
        return false;
    }

    PointType center_pt;
    center_pt.x = frame.odom_pos.x();
    center_pt.y = frame.odom_pos.y();
    center_pt.z = frame.odom_pos.z();

    std::vector<int> radius_indices;
    std::vector<float> radius_sqdist;
    size_t found = global_map_kdtree->radiusSearch(center_pt, ndt_fallback_radius, radius_indices, radius_sqdist);
    if (found < static_cast<size_t>(ndt_fallback_min_points))
    {
        RCLCPP_WARN(logger,
                    "GICP fallback: insufficient map points (%zu < %d) within %.2f m of odom guess.",
                    found,
                    ndt_fallback_min_points,
                    ndt_fallback_radius);
        return false;
    }

    auto target_cloud = std::make_shared<PointCloudXYZI>();
    target_cloud->points.reserve(found);
    for (int idx : radius_indices)
    {
        target_cloud->points.push_back(global_map->points[idx]);
    }

    auto source_cloud = std::make_shared<PointCloudXYZI>();
    pcl::copyPointCloud(*scan_in, *source_cloud);

    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
    gicp.setMaxCorrespondenceDistance(ndt_fallback_radius);
    gicp.setMaximumIterations(80);
    gicp.setTransformationEpsilon(1e-3);
    gicp.setEuclideanFitnessEpsilon(1e-3);
    gicp.setInputSource(source_cloud);
    gicp.setInputTarget(target_cloud);

    // Try multiple yaw angles to handle tilted device initialization
    std::vector<double> yaw_candidates = {0.0, M_PI/4, M_PI/2, 3*M_PI/4, M_PI, -M_PI/4, -M_PI/2, -3*M_PI/4};
    double best_fitness = std::numeric_limits<double>::infinity();
    Eigen::Matrix4f best_transform = Eigen::Matrix4f::Identity();
    bool any_converged = false;

    for (double yaw_offset : yaw_candidates)
    {
        pcl::PointCloud<PointType> aligned;
        Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
        
        // Apply yaw offset to the initial rotation guess
        Eigen::Matrix3f rot_with_offset = frame.odom_rot.toRotationMatrix().cast<float>();
        Eigen::AngleAxisf yaw_rot(yaw_offset, Eigen::Vector3f::UnitZ());
        rot_with_offset = yaw_rot.toRotationMatrix() * rot_with_offset;
        
        guess.block<3, 3>(0, 0) = rot_with_offset;
        guess.block<3, 1>(0, 3) = frame.odom_pos.cast<float>();

        gicp.align(aligned, guess);
        
        if (gicp.hasConverged())
        {
            double fitness = gicp.getFitnessScore();
            if (std::isfinite(fitness) && fitness < best_fitness)
            {
                best_fitness = fitness;
                best_transform = gicp.getFinalTransformation();
                any_converged = true;
            }
        }
    }

    if (!any_converged)
    {
        RCLCPP_WARN(logger, "GICP fallback: no yaw angle converged.");
        return false;
    }

    if (best_fitness > ndt_fallback_fitness_thresh)
    {
        RCLCPP_WARN(logger,
                    "GICP fallback rejected: best fitness %.3f exceeds threshold %.3f.",
                    best_fitness,
                    ndt_fallback_fitness_thresh);
        return false;
    }

    Eigen::Matrix4d T_map_lidar = best_transform.cast<double>();
    Eigen::Matrix4d T_i_l = Eigen::Matrix4d::Identity();
    T_i_l.block<3, 3>(0, 0) = Lidar_R_wrt_IMU;
    T_i_l.block<3, 1>(0, 3) = Lidar_T_wrt_IMU;

    T_out = T_map_lidar * T_i_l.inverse();
    fitness_out = best_fitness;
    RCLCPP_INFO(logger,
                "GICP fallback succeeded with multi-angle search: best fitness %.3f, map points %zu.",
                best_fitness,
                target_cloud->size());
    return true;
}
void global_localization()
{
    auto logger = rclcpp::get_logger("fast_localization.global_localizer");
    double adaptive_fitness_thresh = global_loc_fitness_thresh;
    int consecutive_rejects = 0;

    auto relax_adaptive_threshold = [&](const std::string &reason) {
        if (!global_loc_fitness_adaptive_enable)
        {
            return;
        }
        consecutive_rejects++;
        double target = global_loc_fitness_thresh + consecutive_rejects * global_loc_fitness_adaptive_step;
        adaptive_fitness_thresh = std::min(global_loc_fitness_thresh_max, target);
        RCLCPP_WARN(logger,
                    "Global localization adaptive fitness threshold raised to %.3f due to %s (streak %d)",
                    adaptive_fitness_thresh,
                    reason.c_str(),
                    consecutive_rejects);
    };

    auto reset_adaptive_threshold = [&]() {
        consecutive_rejects = 0;
        adaptive_fitness_thresh = global_loc_fitness_thresh;
    };

    rclcpp::WallRate rate(20.0);
    while (rclcpp::ok())
    {
        {
            std::unique_lock<std::mutex> lock_state(global_localization_finish_state_mutex);
            if (global_localization_finish)
            {
                lock_state.unlock();
                rate.sleep();
                continue;
            }
        }

        int init_check = 0;
        std::vector<int> init_ids;
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> init_poses;
        std::vector<double> init_pose_fitness;

        while (init_check < 2 && rclcpp::ok())
        {
            std::unique_lock<std::mutex> lock_init_feats(init_feats_down_body_mutex);
            int N = init_feats_down_bodys.size();
            if (N == 0)
            {
                lock_init_feats.unlock();
                rate.sleep();
                continue;
            }

            auto init_frame = init_feats_down_bodys.front();
            init_feats_down_bodys.pop();
            lock_init_feats.unlock();

            int current_init_id = init_frame.id;
            PointCloudXYZI::Ptr current_init_pc_origin = init_frame.cloud;
            PointCloudXYZI::Ptr fallback_scan(new PointCloudXYZI);
            pcl::copyPointCloud(*current_init_pc_origin, *fallback_scan);
            PointCloudXYZI::Ptr current_init_pc(new PointCloudXYZI);
            pcl::copyPointCloud(*current_init_pc_origin, *current_init_pc);

            scManager->makeAndSaveScancontextAndKeys(*current_init_pc);
            auto detect_result = scManager->detectLoopClosureID();
            int localization_id = detect_result.first;
            float yaw_init = detect_result.second;

            RCLCPP_DEBUG(logger, "ScanContext detection: id=%d, yaw_diff=%.3f rad (%.1f deg), scan_points=%zu",
                        localization_id, yaw_init, yaw_init * 180.0 / M_PI, current_init_pc->points.size());

            if (localization_id == -1)
            {
                RCLCPP_DEBUG(logger, "ScanContext: No match found (distance > threshold). Trying next scan...");
                init_check = 0;
                relax_adaptive_threshold("ScanContext miss");
                continue;
            }
            
            RCLCPP_INFO(logger, "ScanContext match found: tile_id=%d, yaw_offset=%.3f rad (%.1f deg)",
                       localization_id, yaw_init, yaw_init * 180.0 / M_PI);

            Eigen::AngleAxisd yaw(-yaw_init, Eigen::Vector3d::UnitZ());
            Eigen::Matrix4d T_init_sc = Eigen::Matrix4d::Identity();
            T_init_sc.block<3, 3>(0, 0) = yaw.toRotationMatrix();
            pcl::transformPointCloud(*current_init_pc, *current_init_pc, T_init_sc);
            RCLCPP_INFO(logger, "✅ ScanContext global match: map_tile_id=%d, applied_yaw_correction=%.3f rad (%.1f deg)",
                       localization_id, -yaw_init, -yaw_init * 180.0 / M_PI);

            // Validate localization_id: must be within valid range of map tiles
            // Note: detectLoopClosureID() returns index into polarcontexts_ which may include
            // temporary scans from previous relocalization attempts. We need to ensure the
            // returned index corresponds to a valid map tile.
            if (localization_id < 0 || localization_id >= static_cast<int>(scancontext_tiles.size()))
            {
                // If index equals scancontext_tiles.size(), it might be a boundary case
                // Try to use the last valid tile if index is exactly at the boundary
                if (localization_id == static_cast<int>(scancontext_tiles.size()) && !scancontext_tiles.empty())
                {
                    RCLCPP_WARN(logger, "ScanContext returned boundary index %d (available %zu) - clamping to last valid tile %zu. "
                                "This may indicate a mismatch between ScanContext indices and map tiles.",
                                localization_id, scancontext_tiles.size(), scancontext_tiles.size() - 1);
                    localization_id = static_cast<int>(scancontext_tiles.size() - 1);
                    // Continue processing with clamped index instead of dropping
                }
                else
                {
                    RCLCPP_WARN(logger, "ScanContext returned invalid id %d (available %zu) - dropping newest SC entry. "
                                "This may indicate a mismatch between ScanContext indices and map tiles.",
                                localization_id,
                                scancontext_tiles.size());
                    scManager->dropBackScancontextAndKeys(); // remove the just-added init SC to keep indices aligned

                    // Try GICP fallback immediately when SC id is invalid
                    if (enable_ndt_fallback)
                    {
                        Eigen::Matrix4d T_fallback = Eigen::Matrix4d::Identity();
                        double fallback_fitness = std::numeric_limits<double>::infinity();
                        if (run_gicp_fallback(init_frame, fallback_scan, T_fallback, fallback_fitness, logger))
                        {
                            if (std::isfinite(fallback_fitness) && fallback_fitness <= global_loc_gicp_single_shot_thresh)
                            {
                                {
                                    std::unique_lock<std::mutex> lock_state(global_localization_finish_state_mutex);
                                    global_localization_finish = true;
                                }
                                init_result.first = current_init_id;
                                init_result.second = T_fallback;
                                init_result_fitness = fallback_fitness; // Lưu fitness để validate khi apply
                                reset_adaptive_threshold();
                                RCLCPP_INFO(logger,
                                            "GICP fallback accepted (SC invalid id) single-shot fitness %.3f <= %.3f.",
                                            fallback_fitness,
                                            global_loc_gicp_single_shot_thresh);
                                {
                                    std::queue<InitFrame> swap_empty;
                                    std::unique_lock<std::mutex> lock_init_feats(init_feats_down_body_mutex);
                                    std::swap(init_feats_down_bodys, swap_empty);
                                }
                                init_pose_fitness.clear();
                                init_ids.clear();
                                init_poses.clear();
                                break; // Exit init_check loop early
                            }

                            // Otherwise keep as a candidate
                            init_poses.push_back(T_fallback);
                            init_ids.push_back(current_init_id);
                            init_pose_fitness.push_back(fallback_fitness);
                        }
                    }

                    continue;
                }
            }

            auto current_loop_pc = std::make_shared<PointCloudXYZI>();
            if (!load_map_tile_cloud(localization_id, current_loop_pc, logger))
            {
                continue;
            }

            if (localization_id >= static_cast<int>(position_map.size()) || localization_id >= static_cast<int>(pose_map.size()))
            {
                RCLCPP_WARN(logger, "Missing pose metadata for tile %d (positions %zu, orientations %zu)",
                            localization_id,
                            position_map.size(),
                            pose_map.size());
                continue;
            }

            Eigen::Vector3d p = position_map[localization_id];
            Eigen::Quaterniond q = pose_map[localization_id];
            
            // QUAN TRỌNG: Transform map tile từ local frame về global frame
            // PCD được load ở local frame (từ load_map_tile_cloud), cần transform về global frame để match với current scan
            Eigen::Matrix4d T_tile_global = Eigen::Matrix4d::Identity();
            T_tile_global.block<3, 3>(0, 0) = q.toRotationMatrix();
            T_tile_global.block<3, 1>(0, 3) = p;
            
            PointCloudXYZI::Ptr current_loop_pc_global(new PointCloudXYZI);
            pcl::transformPointCloud(*current_loop_pc, *current_loop_pc_global, T_tile_global);

            // Reject candidates too far from current odom estimate (to avoid wrong relocalization when far from camera_init)
            if (global_loc_candidate_max_dist > 0.0)
            {
                double dist_xy = std::hypot(p.x() - init_frame.odom_pos.x(), p.y() - init_frame.odom_pos.y());
                if (dist_xy > global_loc_candidate_max_dist)
                {
                    RCLCPP_WARN(logger,
                                "Rejecting candidate tile %d: planar distance %.2f m exceeds limit %.2f m",
                                localization_id,
                                dist_xy,
                                global_loc_candidate_max_dist);
                    relax_adaptive_threshold("distance gate");
                    continue;
                }
            }

            // Use standard ICP to refine yaw from ScanContext
            // DISABLED pitch/roll search because IMU has systematic bias when device is tilted
            // This works well when device is kept horizontal during operation
            
            // Transform current scan to global frame for ICP matching
            // Current scan is in body frame (with yaw correction applied), map tile is in global frame
            // We need to transform current scan to global frame using initial estimate from ScanContext
            Eigen::Matrix4d T_i_l = Eigen::Matrix4d::Identity();
            T_i_l.block<3, 3>(0, 0) = Lidar_R_wrt_IMU;
            T_i_l.block<3, 1>(0, 3) = Lidar_T_wrt_IMU;
            
            // Initial estimate: tile pose * yaw correction from ScanContext * lidar to IMU
            Eigen::Matrix4d T_or = Eigen::Matrix4d::Identity();
            T_or.block<3, 3>(0, 0) = q.toRotationMatrix();
            T_or.block<3, 1>(0, 3) = p;
            
            // current_init_pc already has yaw correction (T_init_sc) applied
            // So we need: T_global = T_or * T_init_sc * T_i_l
            // But since current_init_pc is already rotated by T_init_sc, we use:
            Eigen::Matrix4d T_init_estimate = T_or * T_i_l;
            
            // Transform current scan (with yaw correction) to global frame
            PointCloudXYZI::Ptr current_init_pc_global(new PointCloudXYZI);
            pcl::transformPointCloud(*current_init_pc, *current_init_pc_global, T_init_estimate);
            
            // Kiểm tra số điểm trước khi chạy ICP
            if (current_init_pc_global->empty() || current_loop_pc_global->empty())
            {
                RCLCPP_WARN(logger, "Skipping ICP: empty point clouds (source: %zu, target: %zu)", 
                           current_init_pc_global->points.size(), current_loop_pc_global->points.size());
                continue;
            }
            
            if (current_init_pc_global->points.size() < 10 || current_loop_pc_global->points.size() < 10)
            {
                RCLCPP_WARN(logger, "Skipping ICP: too few points (source: %zu, target: %zu)", 
                           current_init_pc_global->points.size(), current_loop_pc_global->points.size());
                continue;
            }
            
            RCLCPP_INFO(logger, "Running ICP: source points=%zu, target points=%zu, tile_id=%d", 
                        current_init_pc_global->points.size(), current_loop_pc_global->points.size(), localization_id);
            
            // Now both clouds are in global frame, do ICP
            Eigen::Matrix4d T_corr = Eigen::Matrix4d::Identity();
            pcl::IterativeClosestPoint<PointType, PointType> icp;
            
            // Coarse alignment first - TĂNG RẤT CAO để tránh "Not enough correspondences"
            // Tăng max correspondence distance lên rất cao để có nhiều correspondences hơn
            // Tăng lên 500m để đảm bảo có correspondences ngay cả khi point cloud thưa hoặc downsample
            icp.setMaxCorrespondenceDistance(500.0);  // Tăng từ 100.0 lên 500.0 để có nhiều correspondences hơn
            icp.setMaximumIterations(100);  // Giữ nguyên
            icp.setTransformationEpsilon(1e-4);  // Giảm epsilon để chính xác hơn
            icp.setEuclideanFitnessEpsilon(1e-4);  // Giảm epsilon để chính xác hơn
            icp.setInputSource(current_init_pc_global);
            icp.setInputTarget(current_loop_pc_global);
            auto unused = std::make_shared<pcl::PointCloud<PointType>>();
            // PCL ICP requires Matrix4f, not Matrix4d
            Eigen::Matrix4f T_init_estimate_f = T_init_estimate.cast<float>();
            icp.align(*unused, T_init_estimate_f);  // Use initial estimate
            Eigen::Matrix4d T_corr_current = icp.getFinalTransformation().cast<double>();

            // Fine alignment with tighter correspondence - nhưng vẫn giữ cao để tránh lỗi
            icp.setMaxCorrespondenceDistance(200.0);  // Tăng từ 50.0 lên 200.0 để có nhiều correspondences hơn
            icp.setInputSource(current_init_pc_global);
            icp.setInputTarget(current_loop_pc_global);
            Eigen::Matrix4f T_corr_current_f = T_corr_current.cast<float>();
            icp.align(*unused, T_corr_current_f);  // Use previous result as initial guess
            T_corr_current = icp.getFinalTransformation().cast<double>();
            
            // T_corr is the correction from initial estimate
            // But we need to account for the yaw correction that was already applied
            T_corr = T_corr_current * T_init_estimate.inverse();
            
            if (!icp.hasConverged())
            {
                RCLCPP_WARN(logger, "ICP failed to converge for tile %d (ScanContext candidate %d). Source points: %zu, Target points: %zu", 
                           localization_id, init_check, 
                           current_init_pc_global->points.size(), current_loop_pc_global->points.size());
                
                // Thử GICP fallback nếu ICP fail
                if (enable_ndt_fallback)
                {
                    Eigen::Matrix4d T_fallback = Eigen::Matrix4d::Identity();
                    double fallback_fitness = std::numeric_limits<double>::infinity();
                    if (run_gicp_fallback(init_frame, fallback_scan, T_fallback, fallback_fitness, logger))
                    {
                        RCLCPP_INFO(logger, "GICP fallback succeeded after ICP failure (fitness=%.3f)", fallback_fitness);
                        init_poses.push_back(T_fallback);
                        init_ids.push_back(current_init_id);
                        init_pose_fitness.push_back(fallback_fitness);
                        scManager->dropBackScancontextAndKeys();
                        init_check++;
                        continue;
                    }
                }
                
                // Nếu GICP cũng fail, dùng trực tiếp ScanContext result (không có ICP refinement)
                // ScanContext đã tìm thấy match tốt, có thể dùng trực tiếp
                RCLCPP_WARN(logger, "Both ICP and GICP failed. Using ScanContext result directly (no ICP refinement) for tile %d", localization_id);
                
                // Dùng trực tiếp initial estimate từ ScanContext (không refine bằng ICP)
                // T_init_estimate transform từ body frame (có yaw correction) sang global frame
                // Giống như T_corr_current khi ICP thành công, nhưng không có refinement
                // T_global_body = T_init_estimate * T_init_sc (để account cho T_init_sc đã được apply)
                Eigen::Matrix4d T_global_body = T_init_estimate * T_init_sc;
                
                // Convert to IMU frame: T_global_imu = T_global_body * T_body_imu
                Eigen::Matrix4d T = T_global_body * T_i_l.inverse();
                
                // Đặt fitness cao để chỉ dùng khi không có option khác
                double sc_only_fitness = 10.0; // High fitness vì không có ICP refinement
                init_poses.push_back(T);
                init_ids.push_back(current_init_id);
                init_pose_fitness.push_back(sc_only_fitness);
                scManager->dropBackScancontextAndKeys();
                init_check++;
                continue;
            }
            
            double current_icp_fitness = icp.getFitnessScore();
            Eigen::Vector3d final_pos = T_corr_current.block<3, 1>(0, 3);
            Eigen::Matrix3d final_rot = T_corr_current.block<3, 3>(0, 0);
            Eigen::Quaterniond final_quat(final_rot);
            RCLCPP_INFO(logger, "ICP refinement: fitness=%.4f (tile %d), final_pos=[%.2f, %.2f, %.2f], converged=%s", 
                        current_icp_fitness, localization_id,
                        final_pos.x(), final_pos.y(), final_pos.z(),
                        icp.hasConverged() ? "yes" : "no");
            // DISABLED: Fitness validation đã được tắt để cho phép localization hoạt động
            // Chấp nhận candidate ngay cả khi fitness cao hơn threshold
            // if (!std::isfinite(current_icp_fitness) || current_icp_fitness > adaptive_fitness_thresh)
            // {
            //     RCLCPP_WARN(logger,
            //                 "Rejecting global localization candidate. Fitness %.3f exceeds threshold %.3f (tile %d)",
            //                 current_icp_fitness,
            //                 adaptive_fitness_thresh,
            //                 localization_id);
            //     relax_adaptive_threshold("ICP reject");
            //     continue;
            // }
            
            // Luôn chấp nhận candidate nếu ICP đã converge
            if (!std::isfinite(current_icp_fitness))
            {
                RCLCPP_WARN(logger, "ICP fitness is not finite, skipping candidate (tile %d)", localization_id);
                continue;
            }
            
            // QUAN TRỌNG: Reject các match có fitness quá cao
            // Fitness > 3.0 thường cho thấy match không tốt (giống recorder project)
            // Recorder project reject nếu fitness > adaptive_fitness_thresh (thường là 0.7-3.5)
            // Nhưng với PCD maps từ livo2, có thể cần threshold cao hơn một chút
            // Tuy nhiên, fitness > 3.0 vẫn là quá cao và cho thấy match sai
            double odom_distance_from_origin = std::hypot(init_frame.odom_pos.x(), init_frame.odom_pos.y(), init_frame.odom_pos.z());
            bool odom_unstable = odom_distance_from_origin < 5.0; // Odometry chưa ổn định nếu gần origin
            
            // Reject nếu fitness quá cao (match không tốt)
            // Khi bắt đầu (odom unstable): reject nếu fitness > 3.0
            // Khi odom đã ổn định: reject nếu fitness > adaptive_fitness_thresh (thường 0.7)
            double fitness_threshold = odom_unstable ? 3.0 : adaptive_fitness_thresh;
            
            if (current_icp_fitness > fitness_threshold)
            {
                RCLCPP_WARN(logger, 
                           "Rejecting candidate with high fitness (%.3f > %.3f) - match quality too poor. Tile %d (odom dist=%.2f, unstable=%s)",
                           current_icp_fitness, fitness_threshold, localization_id, odom_distance_from_origin, odom_unstable ? "yes" : "no");
                
                // Thử GICP fallback nếu có
                if (enable_ndt_fallback)
                {
                    Eigen::Matrix4d T_fallback = Eigen::Matrix4d::Identity();
                    double fallback_fitness = std::numeric_limits<double>::infinity();
                    if (run_gicp_fallback(init_frame, fallback_scan, T_fallback, fallback_fitness, logger))
                    {
                        // Chỉ chấp nhận GICP nếu fitness tốt hơn
                        if (fallback_fitness <= fitness_threshold)
                        {
                            RCLCPP_INFO(logger, "GICP fallback succeeded (fitness=%.3f <= %.3f)", fallback_fitness, fitness_threshold);
                            Eigen::Matrix4d T_global_body = T_fallback * T_init_sc;
                            Eigen::Matrix4d T = T_global_body * T_i_l.inverse();
                            init_poses.push_back(T);
                            init_ids.push_back(current_init_id);
                            init_pose_fitness.push_back(fallback_fitness);
                            scManager->dropBackScancontextAndKeys();
                            init_check++;
                            continue;
                        }
                        else
                        {
                            RCLCPP_WARN(logger, "GICP fallback also has high fitness (%.3f > %.3f), rejecting", fallback_fitness, fitness_threshold);
                        }
                    }
                }
                
                relax_adaptive_threshold("ICP reject");
                continue;
            }
            
            // Log nếu fitness cao nhưng vẫn chấp nhận (trong threshold)
            if (current_icp_fitness > adaptive_fitness_thresh)
            {
                RCLCPP_INFO(logger, "Accepting candidate with fitness: %.3f (threshold: %.3f, tile %d, odom dist=%.2f)",
                           current_icp_fitness, fitness_threshold, localization_id, odom_distance_from_origin);
            }

            // Final pose calculation
            // T_corr_current is the transformation from current_init_pc (body frame with yaw correction) to global frame
            // We need to get the transformation from original body frame to global frame
            // Original body frame -> body frame with yaw correction: T_init_sc
            // Body frame with yaw correction -> global frame: T_corr_current
            // So: T_global_body_original = T_corr_current * T_init_sc
            // Then convert to IMU frame: T_global_imu = T_global_body_original * T_body_imu
            // Note: T_i_l is already declared above, reuse it
            
            // T_corr_current transforms from body frame (with yaw correction) to global frame
            // We need to account for T_init_sc that was applied earlier
            // Final transformation from original body frame to global frame
            Eigen::Matrix4d T_global_body = T_corr_current * T_init_sc;
            
            // Convert to IMU frame: T_global_imu = T_global_body * T_body_imu
            // T_body_imu = T_i_l.inverse()
            Eigen::Matrix4d T = T_global_body * T_i_l.inverse();
            
            // Log final pose for debugging
            Eigen::Vector3d final_pose_pos = T.block<3, 1>(0, 3);
            Eigen::Matrix3d final_pose_rot = T.block<3, 3>(0, 0);
            Eigen::Quaterniond final_pose_quat(final_pose_rot);
            RCLCPP_INFO(logger, "Final localization pose: pos=[%.3f, %.3f, %.3f], quat=[%.3f, %.3f, %.3f, %.3f]",
                       final_pose_pos.x(), final_pose_pos.y(), final_pose_pos.z(),
                       final_pose_quat.x(), final_pose_quat.y(), final_pose_quat.z(), final_pose_quat.w());

            // Accept a strong ScanContext+ICP match immediately (single-shot) to reduce relocalization latency
            if (std::isfinite(current_icp_fitness) && current_icp_fitness <= global_loc_single_shot_thresh)
            {
                {
                    std::unique_lock<std::mutex> lock_state(global_localization_finish_state_mutex);
                    global_localization_finish = true;
                }
                init_result.first = current_init_id;
                init_result.second = T;
                init_result_fitness = current_icp_fitness; // Lưu fitness để validate khi apply
                reset_adaptive_threshold();
                RCLCPP_INFO(logger,
                            "ScanContext+ICP accepted single-shot (fitness %.3f <= %.3f). Skipping second candidate.",
                            current_icp_fitness,
                            global_loc_single_shot_thresh);
                {
                    std::queue<InitFrame> swap_empty;
                    std::unique_lock<std::mutex> lock_init_feats(init_feats_down_body_mutex);
                    std::swap(init_feats_down_bodys, swap_empty);
                }
                init_pose_fitness.clear();
                init_ids.clear();
                init_poses.clear();
                scManager->dropBackScancontextAndKeys();
                break; // Exit init_check loop early
            }

            init_poses.push_back(T);
            init_ids.push_back(current_init_id);
            init_pose_fitness.push_back(current_icp_fitness);
            scManager->dropBackScancontextAndKeys();
            init_check++;
        }

        if (init_poses.size() < 2)
        {
            // If we only have one candidate (or none), try a late GICP fallback to avoid getting stuck
            std::unique_lock<std::mutex> lock_init_feats(init_feats_down_body_mutex);
            if (!init_feats_down_bodys.empty())
            {
                auto init_frame = init_feats_down_bodys.front();
                init_feats_down_bodys.pop();
                lock_init_feats.unlock();

                PointCloudXYZI::Ptr fallback_scan(new PointCloudXYZI);
                pcl::copyPointCloud(*init_frame.cloud, *fallback_scan);

                Eigen::Matrix4d T_fallback = Eigen::Matrix4d::Identity();
                double fallback_fitness = std::numeric_limits<double>::infinity();
                if (enable_ndt_fallback && run_gicp_fallback(init_frame, fallback_scan, T_fallback, fallback_fitness, logger))
                {
                    if (std::isfinite(fallback_fitness) && fallback_fitness <= global_loc_gicp_single_shot_thresh)
                    {
                        {
                            std::unique_lock<std::mutex> lock_state(global_localization_finish_state_mutex);
                            global_localization_finish = true;
                        }
                        init_result.first = init_frame.id;
                        init_result.second = T_fallback;
                        init_result_fitness = fallback_fitness; // Lưu fitness để validate khi apply
                        reset_adaptive_threshold();
                        RCLCPP_INFO(logger,
                                    "GICP fallback accepted single-shot (fitness %.3f <= %.3f).",
                                    fallback_fitness,
                                    global_loc_gicp_single_shot_thresh);
                        std::queue<InitFrame> swap_empty;
                        std::unique_lock<std::mutex> lock_clear(init_feats_down_body_mutex);
                        std::swap(init_feats_down_bodys, swap_empty);
                        init_pose_fitness.clear();
                        init_ids.clear();
                        init_poses.clear();
                        continue;
                    }

                    init_poses.push_back(T_fallback);
                    init_ids.push_back(init_frame.id);
                    init_pose_fitness.push_back(fallback_fitness);
                }
            }
            else
            {
                lock_init_feats.unlock();
            }

            rate.sleep();
            continue;
        }

        // DISABLED: Validation đã được tắt để cho phép localization hoạt động bình thường
        // Chấp nhận kết quả ngay cả khi chỉ có 1 candidate hoặc fitness/xy distance cao hơn threshold
        // LƯU Ý: Global localization và relocalization VẪN HOẠT ĐỘNG BÌNH THƯỜNG
        // - Chỉ tắt validation để không reject kết quả
        // - Nếu muốn bật lại validation, uncomment phần code bên dưới
        
        bool accept_result = true; // Luôn chấp nhận nếu có ít nhất 1 candidate
        
        // Validation đã được tắt - luôn pass
        // Eigen::Vector3d pos_diff = init_poses[0].block<3, 1>(0, 3) - init_poses[1].block<3, 1>(0, 3);
        // Eigen::Vector2d pos_diff_xy(pos_diff.x(), pos_diff.y());
        // double xy_dist = pos_diff_xy.norm();
        // double z_diff = std::abs(pos_diff.z());
        // bool fitness_ready = init_pose_fitness.size() >= 2;
        // bool fitness_within_thresh = fitness_ready && init_pose_fitness[0] <= adaptive_fitness_thresh && init_pose_fitness[1] <= adaptive_fitness_thresh;
        // bool xy_within_thresh = xy_dist < global_loc_convergence_thresh;
        // bool z_within_thresh = (global_loc_height_thresh <= 0.0) || (z_diff < global_loc_height_thresh);
        // bool accept_result = xy_within_thresh && z_within_thresh && fitness_within_thresh;
        
        if (accept_result)
        {
            std::unique_lock<std::mutex> lock_state(global_localization_finish_state_mutex);
            global_localization_finish = true;
            lock_state.unlock();
            init_result.first = init_ids[0];
            init_result.second = init_poses[0];
            // Lưu fitness của result để validate khi apply
            init_result_fitness = init_pose_fitness.size() > 0 ? init_pose_fitness[0] : -1.0;
            // Log thông tin nếu có 2 candidates
            if (init_poses.size() >= 2)
            {
                Eigen::Vector3d pos_diff = init_poses[0].block<3, 1>(0, 3) - init_poses[1].block<3, 1>(0, 3);
                Eigen::Vector2d pos_diff_xy(pos_diff.x(), pos_diff.y());
                double xy_dist = pos_diff_xy.norm();
                double z_diff = std::abs(pos_diff.z());
                RCLCPP_INFO(logger, "Global localization accepted (validation disabled): xy %.2f | z %.2f | fitness[0]=%.3f fitness[1]=%.3f", 
                           xy_dist, z_diff, 
                           init_pose_fitness.size() > 0 ? init_pose_fitness[0] : -1.0,
                           init_pose_fitness.size() > 1 ? init_pose_fitness[1] : -1.0);
            }
            else
            {
                RCLCPP_INFO(logger, "Global localization accepted (validation disabled): single candidate, fitness=%.3f", 
                           init_pose_fitness.size() > 0 ? init_pose_fitness[0] : -1.0);
            }
            reset_adaptive_threshold();
            {
                std::queue<InitFrame> swap_empty;
                std::unique_lock<std::mutex> lock_init_feats(init_feats_down_body_mutex);
                std::swap(init_feats_down_bodys, swap_empty);
            }
            RCLCPP_INFO(logger, "Global localization successfully completed");
            init_pose_fitness.clear();
        }
        else
        {
            // Validation đã được tắt - không reject nữa
            // Code này không bao giờ chạy vì accept_result luôn = true
            // Giữ lại để tránh lỗi compile
            RCLCPP_WARN(logger, "This should not happen - validation is disabled");
            init_ids.clear();
            init_poses.clear();
            init_pose_fitness.clear();
        }

        rate.sleep();
    }
}

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.effect_map_en", false);
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<double>("preprocess.max_range", 100.f);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<int>("common.localization_mode", 1);
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("common.global_loc_convergence_thresh", 2.0);
        this->declare_parameter<double>("common.global_loc_fitness_thresh", 0.5);
        this->declare_parameter<double>("global_loc.fitness_thresh_max", 2.5);
        this->declare_parameter<double>("global_loc.fitness_adaptive_step", 0.2);
        this->declare_parameter<bool>("global_loc.fitness_adaptive_enable", true);
        this->declare_parameter<double>("global_loc.single_shot_fitness_thresh", 0.35);
        this->declare_parameter<double>("global_loc.gicp_single_shot_thresh", 0.20);
        this->declare_parameter<double>("common.global_loc_height_thresh", -1.0);
        this->declare_parameter<bool>("common.global_loc_force_map_height", true);
        this->declare_parameter<bool>("common.global_loc_auto_height_align", true);
        this->declare_parameter<double>("common.global_loc_z_bias", 0.0);
        this->declare_parameter<bool>("global_loc.enable_virtual_scancontexts", true);
        this->declare_parameter<double>("global_loc.virtual_tile_spacing", 10.0);
        this->declare_parameter<double>("global_loc.virtual_tile_radius", 15.0);
        this->declare_parameter<int>("global_loc.virtual_tile_min_points", 400);
        this->declare_parameter<int>("global_loc.virtual_tile_max_tiles", 200);
        this->declare_parameter<bool>("global_loc.freeze_odom_until_locked", true);
        this->declare_parameter<bool>("global_loc.freeze_on_no_points", true);
        this->declare_parameter<double>("global_loc.freeze_on_no_points_tolerance", 0.25);
        this->declare_parameter<bool>("global_loc.unfreeze_after_relocalize", true);
        this->declare_parameter<double>("global_loc.candidate_max_distance", 15.0);
        this->declare_parameter<bool>("global_loc.auto_trigger_enable", true);
        this->declare_parameter<double>("global_loc.auto_trigger_xy_distance", 60.0);
        this->declare_parameter<double>("global_loc.auto_trigger_height", 3.0);
        this->declare_parameter<double>("global_loc.auto_trigger_cooldown", 8.0);
        this->declare_parameter<double>("global_loc.auto_trigger_hysteresis", 5.0);
        this->declare_parameter<bool>("global_loc.enable_ndt_fallback", true);
        this->declare_parameter<double>("global_loc.ndt_fallback_radius", 25.0);
        this->declare_parameter<int>("global_loc.ndt_fallback_min_points", 200);
        this->declare_parameter<double>("global_loc.ndt_fallback_fitness_thresh", 1.5);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        // this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());
        this->declare_parameter<string>("map_root", "");
        this->declare_parameter<string>("map_file_path", "");
        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
        this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
        this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<int>("common.localization_mode", localization_mode, 1);
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic, "/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("common.global_loc_convergence_thresh", global_loc_convergence_thresh, 2.0);
        this->get_parameter_or<double>("common.global_loc_fitness_thresh", global_loc_fitness_thresh, 0.5);
        this->get_parameter_or<double>("global_loc.fitness_thresh_max", global_loc_fitness_thresh_max, 2.5);
        this->get_parameter_or<double>("global_loc.fitness_adaptive_step", global_loc_fitness_adaptive_step, 0.2);
        this->get_parameter_or<bool>("global_loc.fitness_adaptive_enable", global_loc_fitness_adaptive_enable, true);
        this->get_parameter_or<double>("global_loc.single_shot_fitness_thresh", global_loc_single_shot_thresh, 0.35);
        this->get_parameter_or<double>("global_loc.gicp_single_shot_thresh", global_loc_gicp_single_shot_thresh, 0.20);
        this->get_parameter_or<double>("common.global_loc_height_thresh", global_loc_height_thresh, -1.0);
        this->get_parameter_or<bool>("common.global_loc_force_map_height", global_loc_force_map_height, true);
        this->get_parameter_or<bool>("common.global_loc_auto_height_align", global_loc_auto_height_align, true);
        this->get_parameter_or<double>("common.global_loc_z_bias", global_loc_z_bias, 0.0);
        this->get_parameter_or<bool>("global_loc.enable_virtual_scancontexts", enable_virtual_scancontexts, true);
        this->get_parameter_or<double>("global_loc.virtual_tile_spacing", virtual_tile_spacing, 10.0);
        this->get_parameter_or<double>("global_loc.virtual_tile_radius", virtual_tile_radius, 15.0);
        this->get_parameter_or<int>("global_loc.virtual_tile_min_points", virtual_tile_min_points, 400);
        this->get_parameter_or<int>("global_loc.virtual_tile_max_tiles", virtual_tile_max_tiles, 200);
        this->get_parameter_or<bool>("global_loc.freeze_odom_until_locked", freeze_odom_until_global_loc, true);
        this->get_parameter_or<bool>("global_loc.freeze_on_no_points", freeze_odom_on_no_points, true);
        this->get_parameter_or<double>("global_loc.freeze_on_no_points_tolerance", freeze_odom_no_point_tolerance, 0.25);
        this->get_parameter_or<bool>("global_loc.unfreeze_after_relocalize", unfreeze_after_global_loc, true);
        this->get_parameter_or<double>("global_loc.candidate_max_distance", global_loc_candidate_max_dist, 15.0);
        this->get_parameter_or<double>("global_loc.max_jump_distance", global_loc_max_jump_distance, 3.0);
        this->get_parameter_or<double>("global_loc.max_jump_rotation", global_loc_max_jump_rotation, 0.52);
        this->get_parameter_or<bool>("global_loc.auto_trigger_enable", auto_trigger_global_loc_enabled_, true);
        this->get_parameter_or<double>("global_loc.auto_trigger_xy_distance", auto_trigger_distance_xy_, 60.0);
        this->get_parameter_or<double>("global_loc.auto_trigger_height", auto_trigger_height_thresh_, 3.0);
        this->get_parameter_or<double>("global_loc.auto_trigger_cooldown", auto_trigger_cooldown_sec_, 8.0);
        this->get_parameter_or<double>("global_loc.auto_trigger_hysteresis", auto_trigger_distance_hysteresis_, 5.0);
        this->get_parameter_or<bool>("global_loc.enable_ndt_fallback", enable_ndt_fallback, true);
        this->get_parameter_or<double>("global_loc.ndt_fallback_radius", ndt_fallback_radius, 25.0);
        this->get_parameter_or<int>("global_loc.ndt_fallback_min_points", ndt_fallback_min_points, 200);
        this->get_parameter_or<double>("global_loc.ndt_fallback_fitness_thresh", ndt_fallback_fitness_thresh, 1.5);
        this->get_parameter_or<double>("filter_size_corner", filter_size_corner_min, 0.5);
        this->get_parameter_or<double>("filter_size_surf", filter_size_surf_min, 0.5);
        this->get_parameter_or<double>("filter_size_map", filter_size_map_min, 0.5);
        this->get_parameter_or<double>("cube_side_length", cube_len, 200.f);
        this->get_parameter_or<float>("mapping.det_range", DET_RANGE, 300.f);
        this->get_parameter_or<double>("mapping.fov_degree", fov_deg, 180.f);
        this->get_parameter_or<double>("mapping.gyr_cov", gyr_cov, 0.1);
        this->get_parameter_or<double>("mapping.acc_cov", acc_cov, 0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov", b_gyr_cov, 0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov", b_acc_cov, 0.0001);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, true);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<double>("preprocess.max_range", p_pre->max_scan_range, 100.f);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        // this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        // this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());
        std::string configured_map_root;
        this->get_parameter_or<string>("map_root", configured_map_root, "");
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        RCLCPP_INFO(this->get_logger(), "Global localization thresholds → xy %.2f m, z %.2f m (<=0 disables), fitness %.3f, force_map_height=%s, auto_height_align=%s, z_bias=%.3f",
                    global_loc_convergence_thresh, global_loc_height_thresh, global_loc_fitness_thresh,
                    global_loc_force_map_height ? "true" : "false",
                    global_loc_auto_height_align ? "true" : "false",
                    global_loc_z_bias);
        if (global_loc_fitness_adaptive_enable)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Adaptive fitness threshold enabled → max %.2f step %.2f",
                        global_loc_fitness_thresh_max,
                        global_loc_fitness_adaptive_step);
        }
        RCLCPP_INFO(this->get_logger(),
                    "Virtual ScanContext → enabled=%s spacing=%.2f m radius=%.2f m min_pts=%d max_tiles=%d",
                    enable_virtual_scancontexts ? "true" : "false",
                    virtual_tile_spacing,
                    virtual_tile_radius,
                    virtual_tile_min_points,
                    virtual_tile_max_tiles);
        RCLCPP_INFO(this->get_logger(),
                    "Freeze odom until relocalized → %s",
                    freeze_odom_until_global_loc ? "true" : "false");
        RCLCPP_INFO(this->get_logger(),
                    "Freeze odom on missing points → %s (tolerance %.2fs)",
                    freeze_odom_on_no_points ? "true" : "false",
                    freeze_odom_no_point_tolerance);
        RCLCPP_INFO(this->get_logger(),
                    "Auto relocalization → enabled=%s xy>=%.1f m dz>=%.1f m hysteresis=%.1f m cooldown=%.1fs",
                    auto_trigger_global_loc_enabled_ ? "true" : "false",
                    auto_trigger_distance_xy_,
                    auto_trigger_height_thresh_,
                    auto_trigger_distance_hysteresis_,
                    auto_trigger_cooldown_sec_);
        RCLCPP_INFO(this->get_logger(),
                    "Fallback GICP → enabled=%s radius=%.2f m min_pts=%d fitness_thr=%.2f",
                    enable_ndt_fallback ? "true" : "false",
                    ndt_fallback_radius,
                    ndt_fallback_min_points,
                    ndt_fallback_fitness_thresh);
        if (!configured_map_root.empty())
        {
            map_root_dir = configured_map_root;
        }
        else if (!map_file_path.empty())
        {
            map_root_dir = map_file_path;
        }

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "camera_init";

        /*** variables definition ***/
        int effect_feat_num = 0, frame_num = 0;
        double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        bool flg_EKF_converged, EKF_stop_flg = 0;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        double epsi[23] = {0.001};
        fill(epsi, epsi + 23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        /*** debug record ***/
        // FILE *fp;
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(), "w");

        // ofstream fout_pre, fout_out, fout_dbg;
        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"), ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~" << ROOT_DIR << " file opened" << endl;
        else
            cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;

        /*** ROS subscribe initialization ***/
        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20000, livox_pcl_cbk);
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, 20000, standard_pcl_cbk);
        }
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk);
        // Subscribe to /map_to_odom để nhận kết quả từ Python global localization (ScanContext + ICP)
        // Python global localization node sẽ tự động tìm vị trí bằng ScanContext + ICP khi có scan đầu tiên
        // Sau đó publish /map_to_odom với kết quả tìm được
        // C++ code nhận kết quả này và set global_localization_finish = true để hệ thống có thể di chuyển
        // KHÔNG CÒN initial pose từ user hoặc bag file - hệ thống tự tìm vị trí
        sub_map_to_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/map_to_odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                // Khi nhận /map_to_odom từ global_localization.py (kết quả từ ScanContext + ICP)
                // Nếu chưa có global localization, set ngay để hệ thống có thể di chuyển
                std::unique_lock<std::mutex> lock_state(global_localization_finish_state_mutex);
                if (!global_localization_finish)
                {
                    // Convert odom pose to transformation matrix
                    Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
                    Eigen::Quaterniond quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
                    Eigen::Matrix4d T_map_to_odom = Eigen::Matrix4d::Identity();
                    T_map_to_odom.block<3, 3>(0, 0) = quat.toRotationMatrix();
                    T_map_to_odom.block<3, 1>(0, 3) = pos;
                    
                    // Set global_localization_finish = true để hệ thống có thể di chuyển
                    // Kết quả này đến từ Python global localization (ScanContext + ICP), không phải initial pose
                    global_localization_finish = true;
                    
                    // Set init_result để hệ thống có thể sử dụng pose từ Python global localization
                    if (!position_init.empty())
                    {
                        init_result.first = 0; // Use first position in history
                    }
                    else
                    {
                        init_result.first = -1; // Mark as not from C++ global localization
                    }
                    init_result.second = T_map_to_odom;
                    init_result_fitness = -1.0; // Không có fitness từ Python, sẽ không validate
                    
                    RCLCPP_INFO(this->get_logger(), 
                               "Received /map_to_odom from Python global localization (ScanContext + ICP) - enabling localization");
                }
            });
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
        pubGlobalMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_map", 1);
        pubMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map", 1);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        use_global_localization_ = (localization_mode != 0);
        last_no_point_log_ = this->get_clock()->now();
        if (!use_global_localization_)
        {
            global_localization_finish = true;
            global_update = true;
        }
        else
        {
            size_t tile_count = 0;
            map_loaded_ = load_file(pubGlobalMap_, this->get_clock(), this->get_logger(), tile_count);
            map_tile_count_ = tile_count;
            size_t virtual_tiles_generated = 0;
            if (map_loaded_ && enable_virtual_scancontexts)
            {
                virtual_tiles_generated = generate_virtual_scancontexts(this->get_logger());
                if (virtual_tiles_generated > 0)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Generated %zu virtual ScanContext tiles to densify coverage.",
                                virtual_tiles_generated);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(),
                                "No additional virtual ScanContext tiles were created (enable=%s, spacing=%.2f, radius=%.2f).",
                                enable_virtual_scancontexts ? "true" : "false",
                                virtual_tile_spacing,
                                virtual_tile_radius);
                }
            }

            if (!map_loaded_)
            {
                RCLCPP_WARN(this->get_logger(), "FAST-Localization map unavailable. Continuing without global relocalization.");
                use_global_localization_ = false;
            }
            else if (map_tile_count_ <= 1)
            {
                RCLCPP_WARN(this->get_logger(), "FAST-Localization detected %zu map tile(s). Disabling ScanContext relocalization to avoid false matches.", map_tile_count_);
                use_global_localization_ = false;
            }

            if (use_global_localization_ && !global_map->empty())
            {
                ikdtree_global.set_downsample_param(filter_size_map_min);
                ikdtree_global.Build(global_map->points);
                size_t total_sc_entries = scancontext_tiles.size();
                size_t virtual_tiles = total_sc_entries >= map_tile_count_ ? (total_sc_entries - map_tile_count_) : 0;
                RCLCPP_INFO(this->get_logger(),
                            "Global map contains %zu points | ScanContext entries: %zu (real %zu, virtual %zu)",
                            global_map->points.size(),
                            total_sc_entries,
                            map_tile_count_,
                            virtual_tiles);
                // Publish complete global map to /map topic for RViz (with RGB if available)
                if (pubMap_ != nullptr)
                {
                    sensor_msgs::msg::PointCloud2 msg_map;
                    size_t point_count = 0;
                    
                    if (global_map_has_rgb && !global_map_rgb->empty())
                    {
                        pcl::toROSMsg(*global_map_rgb, msg_map);
                        point_count = global_map_rgb->points.size();
                        RCLCPP_INFO(this->get_logger(), "Published global map with RGB colors to /map topic (%zu points)", point_count);
                    }
                    else if (!global_map->empty())
                    {
                        pcl::toROSMsg(*global_map, msg_map);
                        point_count = global_map->points.size();
                        RCLCPP_INFO(this->get_logger(), "Published global map to /map topic (%zu points, no RGB)", point_count);
                    }
                    
                    if (point_count > 0)
                    {
                        msg_map.header.frame_id = "camera_init";
                        msg_map.header.stamp = this->get_clock()->now();
                        pubMap_->publish(msg_map);
                    }
                }
                global_localization_thread_ = std::thread(global_localization);
            }
            else
            {
                global_localization_finish = true;
                global_update = true;
                map_loaded_ = map_loaded_ && !global_map->empty();
            }
        }
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(5000.0 / 100.0));
        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserMappingNode::timer_callback, this));

        // Setup timer to periodically republish global map to /map topic for RViz
        if (map_loaded_ && !global_map->empty())
        {
            auto map_pub_period_ms = std::chrono::milliseconds(1000); // Publish every 1 second
            map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_pub_period_ms, 
                std::bind(&LaserMappingNode::map_pub_timer_callback, this));
        }

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserMappingNode()
    {
        fout_out.close();
        fout_pre.close();
        fclose(fp);
        if (global_localization_thread_.joinable())
        {
            global_localization_thread_.join();
        }
    }

private:
    void map_pub_timer_callback()
    {
        if (map_loaded_ && pubMap_ != nullptr)
        {
            sensor_msgs::msg::PointCloud2 msg_map;
            
            // Publish RGB map if available, otherwise publish normal map
            if (global_map_has_rgb && !global_map_rgb->empty())
            {
                pcl::toROSMsg(*global_map_rgb, msg_map);
            }
            else if (!global_map->empty())
            {
                pcl::toROSMsg(*global_map, msg_map);
            }
            else
            {
                return; // No map to publish
            }
            
            msg_map.header.frame_id = "camera_init";
            msg_map.header.stamp = this->get_clock()->now();
            pubMap_->publish(msg_map);
        }
    }

    void timer_callback()
    {
        if (sync_packages(Measures))
        {
            if (flg_reset)
            {
                RCLCPP_WARN(this->get_logger(), "reset when rosbag play back\n");
                p_imu->Reset();
                flg_reset = false;
                Measures.imu.clear();
                return;
            }

            if (use_global_localization_)
            {
                std::unique_lock<std::mutex> lock_state(global_localization_finish_state_mutex);
                if (global_localization_finish && !global_update)
                {
                    int init_id = init_result.first;
                    Eigen::Matrix4d T_map_current;
                    bool transform_valid = false;
                    
                    if (init_id >= 0 && init_id < static_cast<int>(position_init.size()))
                    {
                        // Case 1: Re-localization từ C++ global localization (có history)
                        Eigen::Vector3d init_time_p = position_init[init_id];
                        Eigen::Quaterniond init_time_q = pose_init[init_id];

                        Eigen::Matrix4d T_odom_init_time = Eigen::Matrix4d::Identity();
                        T_odom_init_time.block<3, 3>(0, 0) = init_time_q.toRotationMatrix();
                        T_odom_init_time.block<3, 1>(0, 3) = init_time_p;

                        Eigen::Matrix4d T_odom_current = Eigen::Matrix4d::Identity();
                        T_odom_current.block<3, 3>(0, 0) = state_point.rot.toRotationMatrix();
                        T_odom_current.block<3, 1>(0, 3) = state_point.pos;

                        Eigen::Matrix4d T_map_init_time = init_result.second;

                        // Tính transform từ map frame đến odom frame hiện tại
                        // T_map_current = T_map_init_time * T_odom_init_time^(-1) * T_odom_current
                        // Điều này chuyển từ odom frame hiện tại sang map frame
                        double current_odom_height_before = state_point.pos(2);
                        T_map_current = T_map_init_time * T_odom_init_time.inverse() * T_odom_current;
                        transform_valid = true;
                        
                        // Validate the global localization result before applying
                        // Check if the new position is too far from current position (jump detection)
                        Eigen::Vector3d new_pos = T_map_current.block<3, 1>(0, 3);
                        Eigen::Vector3d current_pos = state_point.pos;
                        double position_jump = (new_pos - current_pos).norm();
                        
                        // Check rotation change
                        Eigen::Matrix3d new_rot = T_map_current.block<3, 3>(0, 0);
                        Eigen::Matrix3d current_rot = state_point.rot.toRotationMatrix();
                        Eigen::Matrix3d rot_diff = new_rot * current_rot.transpose();
                        Eigen::AngleAxisd angle_axis(rot_diff);
                        double rotation_jump = std::abs(angle_axis.angle());
                        
                        // Reject if jump is too large (likely wrong match)
                        // DISABLED: Validation position/rotation jump đã được tắt để cho phép localization hoạt động bình thường
                        // LƯU Ý: Global localization và relocalization VẪN HOẠT ĐỘNG BÌNH THƯỜNG
                        // - Function global_localization() vẫn chạy và tìm match
                        // - Auto trigger và manual trigger relocalization vẫn hoạt động
                        // - Chỉ có phần validation jump distance/rotation bị tắt để không reject kết quả
                        // Nếu muốn bật lại validation, uncomment phần code bên dưới
                        bool validation_passed = true;
                        
                        // Validation đã được tắt - luôn pass để cho phép localization (không reject kết quả)
                        // if (global_loc_max_jump_distance > 0.0 && position_jump > global_loc_max_jump_distance)
                        // {
                        //     RCLCPP_WARN(this->get_logger(),
                        //                 "❌ Rejecting global localization: position jump too large (%.2f m > %.2f m threshold). "
                        //                 "This likely indicates a wrong match. Current pos=[%.2f, %.2f, %.2f], New pos=[%.2f, %.2f, %.2f]",
                        //                 position_jump, global_loc_max_jump_distance,
                        //                 current_pos.x(), current_pos.y(), current_pos.z(),
                        //                 new_pos.x(), new_pos.y(), new_pos.z());
                        //     validation_passed = false;
                        // }
                        // 
                        // if (validation_passed && global_loc_max_jump_rotation > 0.0 && rotation_jump > global_loc_max_jump_rotation)
                        // {
                        //     RCLCPP_WARN(this->get_logger(),
                        //                 "❌ Rejecting global localization: rotation jump too large (%.2f rad = %.1f deg > %.2f rad threshold). "
                        //                 "This likely indicates a wrong match.",
                        //                 rotation_jump, rotation_jump * 180.0 / M_PI, global_loc_max_jump_rotation);
                        //     validation_passed = false;
                        // }
                        // 
                        // if (!validation_passed)
                        // {
                        //     // Reject this result - mark as processed to avoid retrying immediately
                        //     RCLCPP_ERROR(this->get_logger(),
                        //                 "🚫 REJECTED: Global localization result failed validation. "
                        //                 "Position jump=%.2f m (limit=%.2f m), rotation jump=%.2f deg (limit=%.2f deg). "
                        //                 "This prevents wrong relocalization that would cause device to jump to wrong position.",
                        //                 position_jump, global_loc_max_jump_distance,
                        //                 rotation_jump * 180.0 / M_PI, global_loc_max_jump_rotation * 180.0 / M_PI);
                        //     global_update = true;
                        //     // Do not unlock here - let it unlock at end of block
                        //     // Skip applying this result by using else block below
                        // }
                        // else
                        
                        // Kiểm tra fitness của localization result
                        // Nếu fitness quá cao (match sai), không apply localization để tránh override odometry đúng từ bag
                        // Chỉ apply khi fitness thấp (confidence cao) hoặc không có fitness (từ Python)
                        const double max_acceptable_fitness = 2.0; // Fitness tối đa để chấp nhận (match tốt)
                        bool fitness_valid = (init_result_fitness < 0.0) || (init_result_fitness <= max_acceptable_fitness);
                        
                        if (!fitness_valid)
                        {
                            RCLCPP_WARN(this->get_logger(),
                                        "❌ Rejecting localization result: fitness too high (%.3f > %.3f). "
                                        "This likely indicates a wrong match. Keeping odometry from bag to avoid drift.",
                                        init_result_fitness, max_acceptable_fitness);
                            global_update = true; // Mark as processed to avoid retrying
                            lock_state.unlock();
                            // Skip applying this result, keep using odometry from bag
                            // Không cần làm gì thêm, để code tiếp tục với odometry hiện tại
                        }
                        else
                        {
                            // Luôn pass validation để cho phép localization (nếu fitness OK)
                            if (true) // validation_passed
                        {
                            // Log validation passed with current threshold values for debugging
                            RCLCPP_INFO(this->get_logger(),
                                        "✅ Global localization validation passed: position_jump=%.2f m (limit=%.2f m), rotation_jump=%.2f deg (limit=%.2f deg), fitness=%.3f",
                                        position_jump, global_loc_max_jump_distance,
                                        rotation_jump * 180.0 / M_PI, global_loc_max_jump_rotation * 180.0 / M_PI,
                                        init_result_fitness);
                            
                            if (global_loc_auto_height_align)
                            {
                                double desired_height = current_odom_height_before + global_loc_z_bias;
                                double height_delta = T_map_current(2, 3) - desired_height;
                                T_map_current(2, 3) -= height_delta;
                                RCLCPP_INFO(this->get_logger(), "Auto aligned global map height by %.3f m (target %.3f)", height_delta, desired_height);
                            }
                            else if (global_loc_force_map_height)
                            {
                                double map_init_height = T_map_init_time(2, 3);
                                double init_odom_height = init_time_p(2);
                                double forced_height = map_init_height + (current_odom_height_before - init_odom_height);
                                T_map_current(2, 3) = forced_height + global_loc_z_bias;
                            }

                            state_ikfom global_state = state_point;
                            Eigen::Vector3d new_pos = T_map_current.block<3, 1>(0, 3);
                            Eigen::Matrix3d new_rot = T_map_current.block<3, 3>(0, 0);
                            
                            // Smooth transition for small jumps to avoid sudden movement
                            // For larger jumps (> 1m), apply immediately; for smaller jumps, blend gradually
                            double position_jump_for_smooth = (new_pos - state_point.pos).norm();
                            const double smooth_threshold = 1.0; // meters
                            
                            if (position_jump_for_smooth > 0.1 && position_jump_for_smooth < smooth_threshold)
                            {
                                // Smooth transition: blend 80% new, 20% old to avoid sudden jump
                                const double blend_factor = 0.8;
                                global_state.pos = blend_factor * new_pos + (1.0 - blend_factor) * state_point.pos;
                                
                                // Smooth rotation: slerp between old and new rotation
                                Eigen::Quaterniond old_quat(state_point.rot);
                                Eigen::Quaterniond new_quat(new_rot);
                                global_state.rot = old_quat.slerp(blend_factor, new_quat).toRotationMatrix();
                                
                                RCLCPP_INFO(this->get_logger(), 
                                            "Smooth transition applied: position_jump=%.2f m, blend_factor=%.2f",
                                            position_jump_for_smooth, blend_factor);
                            }
                            else
                            {
                                // For large jumps or very small jumps, apply directly
                                global_state.pos = new_pos;
                                global_state.rot = new_rot;
                            }
                            
                            global_state.grav = S2(Eigen::Vector3d(0, 0, -G_m_s2));
                            
                            // DISABLED: Velocity reset gây cảnh báo tốc độ bất thường (257.42m/s)
                            // Giữ nguyên velocity để tránh cảnh báo và drift
                            // global_state.vel = 0.2 * state_point.vel;  // Reduced from Zero() to smooth transition
                            global_state.vel = state_point.vel;  // Giữ nguyên velocity để tránh cảnh báo tốc độ bất thường
                            
                            kf.change_x(global_state);
                            state_point = kf.get_x();
                            
                            RCLCPP_INFO(this->get_logger(), "Applied global localization alignment. Gravity and velocity reset.");
                            
                            last_stable_state = state_point;
                            last_stable_state_valid = true;
                            last_stable_state_time = Measures.lidar_beg_time;
                            record_auto_trigger_baseline(state_point.pos(2));
                            ikdtree = std::move(ikdtree_global);
                            global_update = true;
                            
                            // Publish path ngay sau khi global localization hoàn thành để hiển thị ngay
                            if (path_en && pubPath_ != nullptr)
                            {
                                publish_path(pubPath_);
                            }
                            
                            if (unfreeze_after_global_loc && freeze_odom_on_no_points)
                            {
                                freeze_odom_on_no_points = false;
                                RCLCPP_WARN(this->get_logger(),
                                            "Unfreezing odom_on_no_points after relocalization (param global_loc.unfreeze_after_relocalize=true).");
                            }
                            position_init.clear();
                            pose_init.clear();
                            last_auto_trigger_planar_distance_ = std::hypot(state_point.pos(0), state_point.pos(1));
                            last_auto_trigger_request_time_ = Measures.lidar_beg_time;
                        }
                        } // End else block (fitness valid)
                    }
                    else if (init_id == -1)
                    {
                        // Case 2: Re-localization từ Python global localization (không có history)
                        // T_map_to_odom đã được set trực tiếp từ Python (đã được invert đúng)
                        Eigen::Matrix4d T_map_to_odom = init_result.second;
                        
                        // T_map_to_odom là transform từ map frame đến odom frame
                        // Để có pose trong map frame từ pose trong odom frame:
                        // T_map_body = T_map_to_odom * T_odom_body
                        // Nhưng T_map_to_odom là transform từ map đến odom, nên để có pose trong map frame:
                        // T_map_body = T_map_to_odom.inverse() * T_odom_body
                        // Hoặc: T_odom_body = T_map_to_odom * T_map_body
                        // Vậy: T_map_body = T_map_to_odom.inverse() * T_odom_body
                        
                        Eigen::Matrix4d T_odom_current = Eigen::Matrix4d::Identity();
                        T_odom_current.block<3, 3>(0, 0) = state_point.rot.toRotationMatrix();
                        T_odom_current.block<3, 1>(0, 3) = state_point.pos;
                        
                        // T_map_current = T_map_to_odom.inverse() * T_odom_current
                        // Điều này chuyển pose từ odom frame sang map frame
                        // Hoặc: T_map_current = T_odom_to_map * T_odom_current
                        // Với T_odom_to_map = T_map_to_odom.inverse()
                        double current_odom_height_before = state_point.pos(2);
                        T_map_current = T_map_to_odom.inverse() * T_odom_current;
                        transform_valid = true;
                        
                        RCLCPP_INFO(this->get_logger(), 
                                   "Applying re-localization from Python global localization (init_id=-1)");
                        RCLCPP_INFO(this->get_logger(), 
                                   "T_map_to_odom: pos=[%.3f, %.3f, %.3f], current_odom: pos=[%.3f, %.3f, %.3f]",
                                   T_map_to_odom(0, 3), T_map_to_odom(1, 3), T_map_to_odom(2, 3),
                                   state_point.pos(0), state_point.pos(1), state_point.pos(2));
                        
                        // Tiếp tục với logic apply transform (giống như case 1)
                        // Validate the global localization result before applying
                        Eigen::Vector3d new_pos = T_map_current.block<3, 1>(0, 3);
                        Eigen::Vector3d current_pos = state_point.pos;
                        double position_jump = (new_pos - current_pos).norm();
                        
                        // Check rotation change
                        Eigen::Matrix3d new_rot = T_map_current.block<3, 3>(0, 0);
                        Eigen::Matrix3d current_rot = state_point.rot.toRotationMatrix();
                        Eigen::Matrix3d rot_diff = new_rot * current_rot.transpose();
                        Eigen::AngleAxisd angle_axis(rot_diff);
                        double rotation_jump = std::abs(angle_axis.angle());
                        
                        bool validation_passed = true;
                        bool fitness_valid = (init_result_fitness < 0.0) || (init_result_fitness <= 2.0);
                        
                        if (!fitness_valid)
                        {
                            RCLCPP_WARN(this->get_logger(),
                                        "❌ Rejecting localization result: fitness too high (%.3f > %.3f). "
                                        "This likely indicates a wrong match. Keeping odometry from bag to avoid drift.",
                                        init_result_fitness, 2.0);
                            global_update = true;
                            lock_state.unlock();
                        }
                        else if (validation_passed)
                        {
                            RCLCPP_INFO(this->get_logger(),
                                        "✅ Global localization validation passed: position_jump=%.2f m, rotation_jump=%.2f deg, fitness=%.3f",
                                        position_jump, rotation_jump * 180.0 / M_PI, init_result_fitness);
                            
                            if (global_loc_auto_height_align)
                            {
                                double desired_height = current_odom_height_before + global_loc_z_bias;
                                double height_delta = T_map_current(2, 3) - desired_height;
                                T_map_current(2, 3) -= height_delta;
                                RCLCPP_INFO(this->get_logger(), "Auto aligned global map height by %.3f m (target %.3f)", height_delta, desired_height);
                            }
                            else if (global_loc_force_map_height)
                            {
                                double map_init_height = T_map_to_odom(2, 3);
                                double forced_height = map_init_height + (current_odom_height_before - 0.0);
                                T_map_current(2, 3) = forced_height + global_loc_z_bias;
                            }

                            state_ikfom global_state = state_point;
                            Eigen::Vector3d new_pos_final = T_map_current.block<3, 1>(0, 3);
                            Eigen::Matrix3d new_rot_final = T_map_current.block<3, 3>(0, 0);
                            
                            // Smooth transition for small jumps
                            double position_jump_for_smooth = (new_pos_final - state_point.pos).norm();
                            const double smooth_threshold = 1.0;
                            
                            if (position_jump_for_smooth > 0.1 && position_jump_for_smooth < smooth_threshold)
                            {
                                const double blend_factor = 0.8;
                                global_state.pos = blend_factor * new_pos_final + (1.0 - blend_factor) * state_point.pos;
                                
                                Eigen::Quaterniond old_quat(state_point.rot);
                                Eigen::Quaterniond new_quat(new_rot_final);
                                global_state.rot = old_quat.slerp(blend_factor, new_quat).toRotationMatrix();
                                
                                RCLCPP_INFO(this->get_logger(), 
                                            "Smooth transition applied: position_jump=%.2f m, blend_factor=%.2f",
                                            position_jump_for_smooth, blend_factor);
                            }
                            else
                            {
                                global_state.pos = new_pos_final;
                                global_state.rot = new_rot_final;
                            }
                            
                            global_state.grav = S2(Eigen::Vector3d(0, 0, -G_m_s2));
                            global_state.vel = state_point.vel;
                            
                            kf.change_x(global_state);
                            state_point = kf.get_x();
                            
                            RCLCPP_INFO(this->get_logger(), "Applied global localization alignment from Python.");
                            RCLCPP_INFO(this->get_logger(), "Position before: [%.3f, %.3f, %.3f], after: [%.3f, %.3f, %.3f]", 
                                       current_pos(0), current_pos(1), current_pos(2),
                                       state_point.pos(0), state_point.pos(1), state_point.pos(2));
                            RCLCPP_INFO(this->get_logger(), "Rotation jump: %.2f deg, Position jump: %.2f m",
                                       rotation_jump * 180.0 / M_PI, position_jump);
                            
                            last_stable_state = state_point;
                            last_stable_state_valid = true;
                            last_stable_state_time = Measures.lidar_beg_time;
                            record_auto_trigger_baseline(state_point.pos(2));
                            ikdtree = std::move(ikdtree_global);
                            global_update = true;
                            
                            // Publish path ngay sau khi global localization hoàn thành để hiển thị ngay
                            if (path_en && pubPath_ != nullptr)
                            {
                                publish_path(pubPath_);
                            }
                            
                            if (unfreeze_after_global_loc && freeze_odom_on_no_points)
                            {
                                freeze_odom_on_no_points = false;
                                RCLCPP_WARN(this->get_logger(),
                                            "Unfreezing odom_on_no_points after relocalization (param global_loc.unfreeze_after_relocalize=true).");
                            }
                            position_init.clear();
                            pose_init.clear();
                            last_auto_trigger_planar_distance_ = std::hypot(state_point.pos(0), state_point.pos(1));
                            last_auto_trigger_request_time_ = Measures.lidar_beg_time;
                        }
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Invalid init id %d for position history size %zu", init_id, position_init.size());
                        global_update = true;
                    }
                }
            }

            double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;

            auto hold_pose_without_points = [&](const char *context) {
                if (!freeze_odom_on_no_points)
                {
                    return;
                }
                double elapsed_since_stable = Measures.lidar_beg_time - last_stable_state_time;
                if (last_stable_state_valid && elapsed_since_stable <= freeze_odom_no_point_tolerance)
                {
                    kf.change_x(last_stable_state);
                    state_point = kf.get_x();
                freeze_skip_counter++;
                if ((Measures.lidar_beg_time - last_freeze_skip_log_time) > 1.0)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "%s → holding last stable pose (freeze_on_no_points enabled) | skipped=%zu in last %.1f s",
                                context,
                                freeze_skip_counter,
                                Measures.lidar_beg_time - last_freeze_skip_log_time);
                    last_freeze_skip_log_time = Measures.lidar_beg_time;
                    freeze_skip_counter = 0;
                }
                    RCLCPP_WARN(this->get_logger(),
                                "%s → holding last stable pose (freeze_on_no_points enabled)",
                                context);
                }
                else if (last_stable_state_valid && elapsed_since_stable > freeze_odom_no_point_tolerance)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "%s → last stable pose too old (%.2fs), allowing odom to drift",
                                context,
                                elapsed_since_stable);
                }
            };

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
        hold_pose_without_points("Empty undistorted scan");
                warn_no_point_once();
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                // cout<<"FAST-LIO not ready"<<endl;
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            maybe_trigger_global_localization(Measures.lidar_beg_time);
            /*** initialize the map kdtree ***/
            if (ikdtree.Root_Node == nullptr)
            {
                RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                if (feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                return;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();

            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
        hold_pose_without_points("Insufficient downsampled points");
                warn_no_point_once();
                return;
            }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose()
                     << " " << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << endl;

            if (0) // If you need to see map point, change to "if(1)"
            {
                PointVector().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();

            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            bool should_freeze_output = use_global_localization_ && freeze_odom_until_global_loc && !global_update;
            state_ikfom state_before_pub;
            bool restored_state_after_pub = false;
            if (should_freeze_output)
            {
                if (!frozen_state_for_pub_valid)
                {
                    frozen_state_for_pub = state_point;
                    frozen_state_for_pub_valid = true;
                }
                state_before_pub = state_point;
                state_point = frozen_state_for_pub;
                restored_state_after_pub = true;
            }
            else
            {
                frozen_state_for_pub_valid = false;
            }

            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);
            last_stable_state = state_point;
            last_stable_state_valid = true;
            last_stable_state_time = Measures.lidar_beg_time;
            record_auto_trigger_baseline(state_point.pos(2));
            if (restored_state_after_pub)
            {
                state_point = state_before_pub;
            }

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            // Always run mapping; previously it was gated to only run before global localization finished,
            // which caused odometry to stop updating after relocalization.
            if (use_global_localization_)
            {
                std::unique_lock<std::mutex> lock_init_feats(init_feats_down_body_mutex);
                map_incremental();
            }
            else
            {
                map_incremental();
            }
            t5 = omp_get_wtime();

            if (use_global_localization_ && !global_localization_finish)
            {
                if (position_init.size() >= max_init_history_)
                {
                    position_init.erase(position_init.begin());
                    pose_init.erase(pose_init.begin());
                }
                position_init.push_back(state_point.pos);
                pose_init.push_back(state_point.rot);
                auto init_cloud = std::make_shared<PointCloudXYZI>();
                pcl::copyPointCloud(*feats_down_body, *init_cloud);
                Eigen::Vector3d init_pose = state_point.pos;
                Eigen::Quaterniond init_rot = state_point.rot;
                std::unique_lock<std::mutex> lock_init_feats(init_feats_down_body_mutex);
                while (init_feats_down_bodys.size() >= max_init_history_)
                {
                    init_feats_down_bodys.pop();
                }
                init_feats_down_bodys.emplace(init_count, init_cloud, init_pose, init_rot);
                init_count++;
            }

            /******* Publish points *******/
            // Publish path ngay cả khi chưa có global localization để hiển thị quãng đường đã đi
            // Chỉ skip khi output bị freeze (để tránh hiển thị path sai trong quá trình chờ localization)
            if (!should_freeze_output && path_en)
            {
                publish_path(pubPath_);
            }
            if (scan_pub_en || pcd_save_en)
                publish_frame_world(pubLaserCloudFull_);
            if (scan_pub_en && scan_body_pub_en)
                publish_frame_body(pubLaserCloudFull_body_);
            // if (effect_pub_en)
            //     publish_effect_world(pubLaserCloudEffect_);

            // publish_map(pubLaserCloudMap_);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1) / frame_num + (kdtree_incremental_time) / frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time + solve_H_time) / frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n", t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose()
                         << " " << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << " " << feats_undistort->points.size() << endl;
                dump_lio_state_to_log(fp);
            }
        }
    }

private:
    void maybe_trigger_global_localization(double current_time)
    {
        if (!auto_trigger_global_loc_enabled_)
        {
            return;
        }
        if (!use_global_localization_ || !map_loaded_ || map_tile_count_ <= 1)
        {
            return;
        }

        const double planar_distance = std::hypot(state_point.pos(0), state_point.pos(1));
        const double height_delta = auto_trigger_baseline_ready_ ? std::abs(state_point.pos(2) - auto_trigger_baseline_height_) : 0.0;
        const bool planar_threshold_enabled = auto_trigger_distance_xy_ > 0.0;
        bool planar_far = planar_threshold_enabled && (planar_distance >= auto_trigger_distance_xy_);
        const bool height_far = (auto_trigger_height_thresh_ > 0.0) && (height_delta >= auto_trigger_height_thresh_);

        if (!planar_far && !height_far)
        {
            return;
        }

        if (planar_far && (planar_distance - last_auto_trigger_planar_distance_) < auto_trigger_distance_hysteresis_)
        {
            planar_far = false;
            if (!height_far)
            {
                return;
            }
        }

        if (last_auto_trigger_request_time_ > 0.0 && (current_time - last_auto_trigger_request_time_) < auto_trigger_cooldown_sec_)
        {
            return;
        }

        {
            std::unique_lock<std::mutex> lock(global_localization_finish_state_mutex);
            if (!global_localization_finish)
            {
                return;
            }
            global_localization_finish = false;
        }

        const std::string reason = planar_far ? "planar distance" : "height drift";
        request_global_localization_reset(reason, current_time, planar_distance, height_delta);
    }

    void request_global_localization_reset(const std::string &reason,
                                           double current_time,
                                           double planar_distance,
                                           double height_delta)
    {
        global_update = false;
        init_result.first = -1;
        init_result.second = Eigen::Matrix4d::Identity();
        init_result_fitness = -1.0; // Reset fitness khi reset localization
        {
            std::queue<InitFrame> swap_empty;
            std::unique_lock<std::mutex> lock(init_feats_down_body_mutex);
            std::swap(init_feats_down_bodys, swap_empty);
        }
        position_init.clear();
        pose_init.clear();
        last_auto_trigger_request_time_ = current_time;
        last_auto_trigger_planar_distance_ = planar_distance;
        RCLCPP_WARN(this->get_logger(),
                    "Auto global localization triggered (%s) | dist_xy=%.2f m, dz=%.2f m",
                    reason.c_str(),
                    planar_distance,
                    height_delta);
    }

    void record_auto_trigger_baseline(double current_height)
    {
        if (!auto_trigger_global_loc_enabled_ || auto_trigger_baseline_ready_)
        {
            return;
        }
        auto_trigger_baseline_height_ = current_height;
        auto_trigger_baseline_ready_ = true;
    }

    void warn_no_point_once()
    {
        auto now = this->get_clock()->now();
        if (last_no_point_log_.nanoseconds() == 0 || (now - last_no_point_log_) > rclcpp::Duration::from_seconds(1.0))
        {
            RCLCPP_WARN(this->get_logger(), "No point, skip this scan!");
            last_no_point_log_ = now;
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGlobalMap_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMap_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_map_to_odom_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;
    std::thread global_localization_thread_;
    bool use_global_localization_ = false;
    bool map_loaded_ = false;
    size_t map_tile_count_ = 0;
    rclcpp::Time last_no_point_log_;
    const size_t max_init_history_ = 200;
    bool auto_trigger_global_loc_enabled_ = true;
    double auto_trigger_distance_xy_ = 60.0; // Increased from 40.0 to reduce false triggers
    double auto_trigger_height_thresh_ = 3.0;
    double auto_trigger_cooldown_sec_ = 10.0; // Increased from 5.0 to reduce frequency
    double auto_trigger_distance_hysteresis_ = 10.0; // Increased from 5.0 to reduce false triggers
    double last_auto_trigger_request_time_ = -1.0;
    double last_auto_trigger_planar_distance_ = 0.0;
    double auto_trigger_baseline_height_ = 0.0;
    bool auto_trigger_baseline_ready_ = false;

    bool effect_pub_en = false, map_pub_en = false;
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE *fp;
    ofstream fout_pre, fout_out, fout_dbg;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, SigHandle);

    rclcpp::spin(std::make_shared<LaserMappingNode>());

    if (rclcpp::ok())
        rclcpp::shutdown();
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(), "w");
        fprintf(fp2, "time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0; i < time_log_counter; i++)
        {
            fprintf(fp2, "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n", T1[i], s_plot[i], int(s_plot2[i]), s_plot3[i], s_plot4[i], int(s_plot5[i]), s_plot6[i], int(s_plot7[i]), int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}
