#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "hirop_msgs/LoadPCL.h"

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
     * @brief 保存点云数据
     * @param file 全局地址
    */
    bool save(const std::string &file);

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
    static bool on_off;
    static bool scan_switch;
    static bool reset_switch;
    static bool set_begin_pose;

    ros::NodeHandle *nh_;
    ros::ServiceClient load_pcl_client;

    geometry_msgs::PoseStamped begin_pose;
};
