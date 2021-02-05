#pragma once

#include "pose_manager.h"
#include "motion_actuator.h"
#include "control_scan.h"

class ScanFramework
{
public:
    ScanFramework();
    ~ScanFramework();

    /**
     * @brief 开启或关闭扫描程序
    */
    bool HMScan(const bool &on_off);

    /**
     * @brief 自动扫描模式
     * @param lp 扫描的参数
    */
    bool autoScan(const LookingParam &lp);

    /**
     * @brief 以现在的姿态进行旋转
    */
    bool rotate();

    bool loadPose(const std::string& file);

    bool insertPose(int insert, bool is_save=false);

    /**
     * @brief 按输入文件的点位进行扫描
     * @param file 记录点位的文件名
    */
    double motionRcordPose();

    /**
     * @brief 记录现在的机器人的姿态
    */
    bool RecordPose();

    /**
     * @brief 保存缓存的机器人姿态
     * @param file 保存的文件名
    */
    bool savePose(const std::string &file);

    /**
     * @brief 保存扫描的数据
     * @param file 保存的点云文件
    */
    bool saveScanData(const std::string &file);

    bool loadScanData(const std::string &file);

    bool resetScanData();

    bool rmWorkspace();

    void stopMotion();

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg_ptr;
    ros::NodeHandle *nh;

    std::shared_ptr<MotionActuator> ma_ptr;
    std::shared_ptr<PoseManager> pm_ptr;
    std::shared_ptr<ControlScan> cs_ptr;
    const std::string pcd_uri = "/home/fshs/.hirop/data/three_dimensions/";
    std::vector<std::vector<double>> motion_joints_pose;
    std::string pose_file_name;
};
