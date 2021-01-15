#include "control_scan.h"

using namespace std;

bool ControlScan::on_off = false;
bool ControlScan::scan_switch = false;
bool ControlScan::reset_switch = false;
bool ControlScan::set_begin_pose = false;

ControlScan::ControlScan(ros::NodeHandle *n)
    : nh_{n}
{
    ROS_INFO_STREAM("on_off " << on_off << ", scan_switch " << scan_switch << ", reset_switch " << reset_switch);
}

ControlScan::~ControlScan()
{
}

bool ControlScan::bringup()
{
    ROS_INFO_STREAM("scan bringup");
    if (on_off)
        return true;
    on_off = true;
    return true;
}

bool ControlScan::start(const geometry_msgs::PoseStamped &pose)
{
    ROS_INFO_STREAM("scan start");
    if (scan_switch)
        return true;
    scan_switch = true;
    reset_switch = false;
    set_begin_pose = true;
    begin_pose = pose;
    return true;
}

bool ControlScan::pause()
{
    ROS_INFO_STREAM("scan pause");
    if (!scan_switch)
        return true;
    return true;
}

bool ControlScan::save(const std::string &file)
{
    ROS_INFO_STREAM("scan save");
    return true;
}

bool ControlScan::dataReset()
{
    ROS_INFO_STREAM("scan reset");
    reset_switch = true;
    set_begin_pose = false;
    return true;
}

bool ControlScan::close()
{
    ROS_INFO_STREAM("scan close");
    if (!on_off)
        return true;
    on_off = false;
    return true;
}

const geometry_msgs::PoseStamped& ControlScan::getBeginPose() const
{
    return begin_pose;
}
