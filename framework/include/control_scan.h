#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "hirop_msgs/LoadPCL.h"
#include <std_srvs/Empty.h>


class ControlScan
{
public:

    ControlScan(ros::NodeHandle *n_);
    ~ControlScan();

    /**
     * @brief 启动扫描
    */
    bool bringup();

    /**
     * @brief 开始扫描
     * @param pose 开始扫描的姿态
    */
    bool start(const geometry_msgs::PoseStamped &pose);

    /**
     * @brief 暂停
    */
    bool pause();

    /**
     * @brief 暂停后重新开始
    */
   bool resume();

    /**
     * @brief 保存点云数据
     * @param file 全局地址
    */
    bool save(const std::string& file_path);

    /**
     * @brief 清空数据集
    */
    bool resetData();

    /**
     * @brief 关闭扫描
    */
    bool close();

    /**
     * @brief 显示扫描结果
     * @param file 全局地址
    */
    bool show(const std::string& file);

    /**
     * @brief 获取开始扫描的姿态
    */
    const geometry_msgs::PoseStamped& getBeginPose() const;

private:
    template<typename T>
    bool callServer(ros::ServiceClient& client, T& srv);

private:
    ros::ServiceClient pause_rtabmap_client;
    ros::ServiceClient pause_rtabmap_odom_client;
    ros::ServiceClient resume_rtabmap_client;
    ros::ServiceClient resume_rtabmap_odom_client;
    ros::ServiceClient save_rtabmap_pcd_client;
    ros::ServiceClient rtabmap_auto_clear_the_cache_client;
    ros::ServiceClient reset_rtabmap_client;
    ros::ServiceClient reset_rtabmap_odom_client;

private:
    static bool on_off;
    static bool scan_switch;
    static bool reset_switch;
    static bool set_begin_pose;

    ros::NodeHandle *nh_;
    ros::ServiceClient load_pcl_client;

    geometry_msgs::PoseStamped begin_pose;
};
