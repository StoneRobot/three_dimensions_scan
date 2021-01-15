#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class ControlScan
{
public:
    static bool on_off;
    static bool scan_switch;
    static bool reset_switch;
    static bool set_begin_pose;

    ControlScan(ros::NodeHandle *n_);
    ~ControlScan();

    bool bringup();
    bool start(const geometry_msgs::PoseStamped &pose);
    bool pause();
    // bool goOn();
    bool save(const std::string &file);
    bool dataReset();
    bool close();

    const geometry_msgs::PoseStamped& getBeginPose() const;

private:
    ros::NodeHandle *nh_;
    geometry_msgs::PoseStamped begin_pose;
};
