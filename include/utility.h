#pragma once
#ifndef _UTILITY_H_
#define _UTILITY_H_
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <boost/circular_buffer.hpp>
#include <boost/format.hpp>

#include <thread>
#include <mutex>
#include <memory>
#include <iostream>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <vector>

using namespace std;

struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                           (uint16_t, ring, ring)(float, time, time)
)

using PointT = pcl::PointXYZI;
using PointIRT = VelodynePointXYZIRT;

class ParamServer
{
public:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    string odom_child_frame_id;

    // BlockMap(BM) settings
    double downsample_resolution;
    string globalmap_dir;

    // Gridmap settings
    string yaml_path_;
    string pgm_path_;

    // Lidar configuation
    string lidarTopic;
    bool have_ring_time_channel;
    float lidarMinRange;
    float lidarMaxRange;
    string ndt_neighbor_search_method;
    double ndt_resolution;
    double ndt_epsilon;

    // IMU configuation
    string imuTopic;
    double imuAccNoise, imuGyrNoise;
    double gravity;
    int imuFrequency;
    vector<double> extRotV;
    vector<double> L2GtRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    vector<double> L2GtTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;

    Eigen::Matrix3d L2GtRot;
    Eigen::Vector3d extTrans;
    Eigen::Vector3d L2GtTrans;
    Eigen::Quaterniond extQRPY;

    // System settings
    float key_interval, mapqry_interval;

    // Optimization settings
    int active_factors;
    int preserve_factors;
    double odomLinearNoise, odomAngularNoise;
    double penalty_thres;
    double penalty_weight;

    ParamServer() {
        nh.param<string>("/block_localization/odom_child_frame_id", odom_child_frame_id, "velodyne");
        nh.param<string>("/globalmap_server/globalmap_dir", globalmap_dir, "");
        nh.param<double>("/globalmap_server/downsample_resolution", downsample_resolution, 0.1);
        nh.param<string>("/globalmap_server/yaml_path", yaml_path_, "");
        nh.param<string>("/globalmap_server/pgm_path", pgm_path_, "");

        nh.param<string>("/block_localization/lidarTopic", lidarTopic, "/points_raw");
        nh.param<bool>("/block_localization/have_ring_time_channel", have_ring_time_channel, true);
        nh.param<float>("/block_localization/lidarMinRange", lidarMinRange, 0.2);
        nh.param<float>("/block_localization/lidarMaxRange", lidarMaxRange, 100.0);
        nh.param<string>("/block_localization/ndt_neighbor_search_method", ndt_neighbor_search_method, "DIRECT7");
        nh.param<double>("/block_localization/ndt_resolution", ndt_resolution, 1.0);
        nh.param<double>("/block_localization/ndt_epsilon", ndt_epsilon, 0.01);

        nh.param<string>("/block_localization/imuTopic", imuTopic, "/imu_raw");
        nh.param<double>("/block_localization/imuAccNoise", imuAccNoise, 0.01);
        nh.param<double>("/block_localization/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<double>("/block_localization/gravity", gravity, 9.8);
        nh.param<int>("/block_localization/imuFrequency", imuFrequency, 100);
        nh.param<vector<double>>("/block_localization/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("/block_localization/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("/block_localization/extrinsicTrans", extTransV, vector<double>());
        nh.param<vector<double>>("/block_localization/L2gt_t", L2GtTransV, vector<double>());
        nh.param<vector<double>>("/block_localization/L2gt_R", L2GtRotV, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY).inverse();

        L2GtTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(L2GtTransV.data(), 3, 1);
        L2GtRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(L2GtRotV.data(), 3, 3);

        nh.param<float>("/block_localization/key_interval", key_interval, 0.1);
        nh.param<float>("/block_localization/mapqry_interval", mapqry_interval, 1.0);

        nh.param<int>("/block_localization/active_factors", active_factors, 120);
        nh.param<int>("/block_localization/preserve_factors", preserve_factors, 10);
        nh.param<double>("/block_localization/odomLinearNoise", odomLinearNoise, 0.01);
        nh.param<double>("/block_localization/odomAngularNoise", odomAngularNoise, 0.01);
        nh.param<double>("/block_localization/penalty_thres", penalty_thres, 1.0);
        nh.param<double>("/block_localization/penalty_weight", penalty_weight, 1.0);
    }


    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }
};


template<typename T>
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher& thisPub, const T& thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub.getNumSubscribers() != 0)
        thisPub.publish(tempCloud);
    return tempCloud;
}


template<typename T>
double getROSTime(T msg) {
    return msg->header.stamp.toSec();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw) {
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


float pointDistance(PointT p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

#endif