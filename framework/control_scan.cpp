#include "control_scan.h"

#include "rtabmap_ros/AutoExportClouds.h"
#include "rtabmap_ros/AutoClearCache.h"

using namespace std;

bool ControlScan::on_off = false;
bool ControlScan::scan_switch = false;
bool ControlScan::reset_switch = false;
bool ControlScan::set_begin_pose = false;

ControlScan::ControlScan(ros::NodeHandle *n)
    : nh_{n}
{
    ROS_INFO_STREAM("on_off " << on_off << ", scan_switch " << scan_switch << ", reset_switch " << reset_switch);
    load_pcl_client = nh_->serviceClient<hirop_msgs::LoadPCL>("loadPointCloud");

    pause_rtabmap_client = nh_->serviceClient<std_srvs::Empty>("/rtabmap/pause");
    pause_rtabmap_odom_client = nh_->serviceClient<std_srvs::Empty>("/rtabmap/pause_odom");
    resume_rtabmap_client = nh_->serviceClient<std_srvs::Empty>("/rtabmap/resume");
    resume_rtabmap_odom_client = nh_->serviceClient<std_srvs::Empty>("/rtabmap/resume_odom");
    save_rtabmap_pcd_client = nh_->serviceClient<rtabmap_ros::AutoExportClouds>("/rtabmap/auto_export_clouds");
    rtabmap_auto_clear_the_cache_client = nh_->serviceClient<rtabmap_ros::AutoClearCache>("/rtabmap/auto_clear_the_cache");
    reset_rtabmap_client = nh_->serviceClient<std_srvs::Empty>("/rtabmap/reset");
    reset_rtabmap_odom_client = nh_->serviceClient<std_srvs::Empty>("/rtabmap/reset_odom");
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
    std_srvs::Empty srv;
    if (callServer<std_srvs::Empty>(pause_rtabmap_odom_client, srv))
        if (callServer<std_srvs::Empty>(pause_rtabmap_client, srv))
            return true;
    return false;
}

bool ControlScan::resume()
{
    std_srvs::Empty srv;
    if(callServer<std_srvs::Empty>(resume_rtabmap_client, srv))
        if(callServer<std_srvs::Empty>(resume_rtabmap_odom_client, srv))
        return true;
    return false;
}

bool ControlScan::save(const std::string& file_path)
{
    ROS_INFO_STREAM("scan save");
    pause();
    rtabmap_ros::AutoExportClouds srv;
    srv.request.default_path = false;
    srv.request.self_path = file_path;
    if (callServer<rtabmap_ros::AutoExportClouds>(save_rtabmap_pcd_client, srv))
        return true;
    return false;
}

bool ControlScan::resetData()
{
    ROS_INFO_STREAM("scan reset");
    pause();
    std_srvs::Empty srv;
    rtabmap_ros::AutoClearCache clear_cache_srv;
    if (callServer<std_srvs::Empty>(reset_rtabmap_client, srv))
        if (callServer<std_srvs::Empty>(reset_rtabmap_odom_client, srv))
            if(callServer<rtabmap_ros::AutoClearCache>(rtabmap_auto_clear_the_cache_client, clear_cache_srv))
            {
                resume();
                return true;
            }
    return false;
}

bool ControlScan::close()
{
    ROS_INFO_STREAM("scan close");
    if (!scan_switch)
        return true;
    scan_switch = false;
    return true;
}

bool ControlScan::show(const std::string &file)
{
    hirop_msgs::LoadPCL srv;
    srv.request.fileName = file;
    load_pcl_client.call(srv);
    return !srv.response.result;
}

const geometry_msgs::PoseStamped &ControlScan::getBeginPose() const
{
    return begin_pose;
}

template<typename T>
bool ControlScan::callServer(ros::ServiceClient& client, T& srv)
{
    bool flag;
    flag = client.call(srv);
    ROS_INFO_STREAM(client.getService() << ": " << (flag ? "SUCCESS" : "FAILED"));
    return flag;
}