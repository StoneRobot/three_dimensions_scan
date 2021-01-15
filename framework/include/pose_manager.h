#pragma once

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>

#include "hirop_msgs/loadJointsData.h"
#include "hirop_msgs/saveJointData.h"
#include "hirop_msgs/saveDataEnd.h"

#include "hirop_msgs/savePoseData.h"
#include "hirop_msgs/loadPoseData.h"

struct LookingParam
{
    std::string frame_id;
    double h;
    double radius;
    bool fixed_R;
    double R;
    bool fixed_P;
    double P;
    bool fixed_Y;
    double Y;
};

class PoseManager
{
public:
    PoseManager(ros::NodeHandle *n);
    ~PoseManager();

    /**
     * @brief 根据数据生成系列点位
     * @param lp 数据
     * @param out_poses 生成的点位
    */
    void generatorPose(const LookingParam &lp, std::vector<geometry_msgs::PoseStamped> &out_poses);

    /**
     * @brief 加载关节点位
     * @param file[input] 输入的文件名
     * @param joints[out] 输出的关节角
     * @return 是否加载成功
    */
    bool loadPose(const std::string &file, std::vector<std::vector<double>> &joints);

    /**
     * @brief 加载笛卡尔点位
    */
    bool loadPose(const std::string &file, std::vector<geometry_msgs::PoseStamped> &pose);

    /**
     * @brief 记录点位
     * @param joint 需要记录的点位
    */
    bool recordPose(const std::vector<double> &joint);

    /**
     * 记录笛卡尔坐标
    */
    bool recordPose(const geometry_msgs::PoseStamped& pose);

    /**
     * @brief 保存使用记录在缓存区的点位,记录完成后清空缓存区
     * @param file 文件名
    */
    bool savePose(const std::string &file);


private:
    void getYPosition(const double &r, const double &x, double &y1, double &y2);

    void generatePosition(const LookingParam &lp, std::vector<geometry_msgs::PoseStamped> &pose);

    void generateOrientation(const LookingParam &lp, std::vector<geometry_msgs::PoseStamped> &pose);

    void tfQuatRotate(const LookingParam &lp, int vx, int vy, int vz, std::vector<geometry_msgs::PoseStamped> &pose);

private:
    ros::ServiceClient load_pose_client;
    ros::ServiceClient record_joint_pose_client;
    ros::ServiceClient save_pose_client;

    ros::ServiceClient record_cartesian_pose_client;
    ros::ServiceClient load_cartesian_pose_client;

private:
    ros::NodeHandle *nh_;
    const std::string file_uri;
    const double position_step;
    const double rpy_step;
    const double travel;

    std::vector<std::vector<double>> record_joints_vet;

    tf2::Quaternion orientation;
};
