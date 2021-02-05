#pragma once

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/Constraints.h>

#include "hirop_msgs/PubObject.h"
#include "hirop_msgs/removeObject.h"

struct Workspace
{
    double max_x;
    double min_x;
    double max_y;
    double min_y;
    double max_z;
    double min_z;
    std::string frame_id;
    std::string tcp;
};

class MotionActuator
{
public:
    MotionActuator(ros::NodeHandle *n, moveit::planning_interface::MoveGroupInterface *move_group);
    ~MotionActuator();

    void setPose(const std::vector<double> &joint);
    void setPose(const std::vector<geometry_msgs::PoseStamped> &poses);

    /**
     * @brief 进行规划,运动,并环视
    */
    bool autoMotion();

    /**
     * @brief 以现在的姿态进行环视
    */
    bool rotate();

    /**
     * @brief 去到对向量里面的动作,然后进行环视
     * @param joints 姿态
     * @return 完成了向量里面的多少个动作,区间[0, 1]
    */
    double motionRecordPose(const std::vector<std::vector<double>> &joints);

    const std::vector<double> getJointPose() const;

    const geometry_msgs::PoseStamped getPose() const;

    /**
     * @brief 进行规划和移动
    */
    bool planAndMove();
    
    bool rmWorkspace();

    void stopMotion();
private:
    /**
     * @brief 进行规划
    */
    bool plan();

    /**
     * @brief 进行移动
    */
    bool move();

    /**
     * @brief 设置约束
     * @param is_set_workspace 是否设置工作空间
     * @param is_set_joint_con 是否设置关节约束
    */
    void setConstraint(const bool &is_set_workspace, const bool &is_set_joint_con);

    /**
     * @brief 设置工作空间的参数
     * @param con[out] 设置空间的参数
    */
    void getWorkspace(moveit_msgs::Constraints &con);

    /**
     * @brief 设置关节约束的参数
     * @param con[out] 设置关节约束参数         
    */
    void getJointConstraint(moveit_msgs::Constraints &con);


private:
    ros::NodeHandle *nh_;
    ros::ServiceClient load_obj_client;
    ros::ServiceClient rm_obj_client;
    moveit::planning_interface::MoveGroupInterface *move_group_;

    // public:
private:
    Workspace ws_;
    const int plan_count;
    const int execute_count;
    const moveit::core::VariableBounds joint_1_bounds;
    std::vector<std::string> ob_name = {"q", "h", "z", "y", "s", "x"};
    bool stop_flag;
};
