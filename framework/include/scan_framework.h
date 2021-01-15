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

    /**
     * @brief 按输入文件的点位进行扫描
     * @param file 记录点位的文件名
    */
    double motionRcordPose(const std::string &file);

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

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg_ptr;
    ros::NodeHandle *nh;

    std::shared_ptr<MotionActuator> ma_ptr;
    std::shared_ptr<PoseManager> pm_ptr;
    std::shared_ptr<ControlScan> cs_ptr;
};
