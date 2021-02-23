#include "control_scan.h"

#include "rtabmap_ros/AutoExportClouds.h"
#include "rtabmap_ros/AutoClearCache.h"

#include "hirop/perception/transformUltity.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "pcl/common/io.h"
#include "pcl/common/transforms.h"

#include "pcl/conversions.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/io/ply_io.h>
#include <tf2_eigen/tf2_eigen.h>

#include <sensor_msgs/PointCloud2.h>
#include <moveit_msgs/PlanningScene.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

using namespace std;

bool ControlScan::on_off = false;
bool ControlScan::scan_switch = false;
bool ControlScan::reset_switch = false;
bool ControlScan::set_begin_pose = false;

ControlScan::ControlScan(ros::NodeHandle *n)
    : nh_{n}
{
    // ROS_INFO_STREAM("on_off " << on_off << ", scan_switch " << scan_switch << ", reset_switch " << reset_switch);
    load_pcl_client = nh_->serviceClient<hirop_msgs::LoadPCL>("loadPointCloud");

    pause_rtabmap_client = nh_->serviceClient<std_srvs::Empty>("/rtabmap/pause");
    pause_rtabmap_odom_client = nh_->serviceClient<std_srvs::Empty>("/rtabmap/pause_odom");
    resume_rtabmap_client = nh_->serviceClient<std_srvs::Empty>("/rtabmap/resume");
    resume_rtabmap_odom_client = nh_->serviceClient<std_srvs::Empty>("/rtabmap/resume_odom");
    save_rtabmap_pcd_client = nh_->serviceClient<rtabmap_ros::AutoExportClouds>("/rtabmap/auto_export_clouds");
    rtabmap_auto_clear_the_cache_client = nh_->serviceClient<rtabmap_ros::AutoClearCache>("/rtabmap/auto_clear_the_cache");
    reset_rtabmap_client = nh_->serviceClient<std_srvs::Empty>("/rtabmap/reset");
    reset_rtabmap_odom_client = nh_->serviceClient<std_srvs::Empty>("/rtabmap/reset_odom");

    pcl_pub = nh_->advertise<sensor_msgs::PointCloud2>("pcl_output", 1);
    publishOctomapPub = nh_->advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);

    start();
}

ControlScan::~ControlScan()
{
}

bool ControlScan::bringup()
{
    ROS_INFO_STREAM("scan bringup");
    // if (on_off)
    //     return true;
    // on_off = true;
    return true;
}

bool ControlScan::start()
{
    string cameraFrame;
    nh_->param("/cameraFrame", cameraFrame, string("point_cloud_pub_frame"));
    transformUltity::transformFrame("world", cameraFrame, t, r);
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
    if (callServer<std_srvs::Empty>(resume_rtabmap_client, srv))
        if (callServer<std_srvs::Empty>(resume_rtabmap_odom_client, srv))
            return true;
    return false;
}

bool ControlScan::save(const std::string &file_path)
{
    ROS_INFO_STREAM("scan save");
    pause();
    string file_pcd = file_path;
    rtabmap_ros::AutoExportClouds srv;
    srv.request.default_path = false;
    srv.request.self_path = file_path;
    if (callServer<rtabmap_ros::AutoExportClouds>(save_rtabmap_pcd_client, srv))
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGB>);
        file_pcd += ".pcd";
        pcl::io::loadPCDFile(file_pcd, *(cloud_.get()));
        Eigen::Isometry3d trans_matrix = transformUltity::makeMatrix(t, r);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*cloud_, *temp, trans_matrix.matrix());
        pcl::io::savePCDFile(file_pcd, *temp);
        return true;
    }
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
            if (callServer<rtabmap_ros::AutoClearCache>(rtabmap_auto_clear_the_cache_client, clear_cache_srv))
            {
                resume();
                start();
                return true;
            }
    return false;
}

bool ControlScan::close()
{
    ROS_INFO_STREAM("scan close");
    // if (!scan_switch)
    //     return true;
    // scan_switch = false;
    return true;
}

// bool PointConvertOctompRBG(octomap::ColorOcTree &tree, pcl::PointCloud<pcl::PointXYZRGB> &cloud_trans)
// {
//     /**
//       pcl转octomap
//     */
//     std::cout << "copy data into octomap..." << std::endl;
//     // octomap::ColorOcTree tree( 0.05 );
//     tree.clear();

//     if (cloud_trans.size() == 0)
//     {
//         cout << "点云过滤后数量为0,无法八叉树化,请重新设置点云过滤配置参数" << endl;
//         return false;
//     }

//     //将点云的点插入到八叉树种
//     // for (auto p:*point.get()){
//     for (auto p : cloud_trans.points)
//     {
//         tree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
//     }

//     //设置颜色信息
//     // for (auto p:*point.get()){
//     for (auto p : cloud_trans.points)
//     {
//         tree.integrateNodeColor(p.x, p.y, p.z, p.r, p.g, p.b);
//     }

//     //更新octomap
//     tree.updateInnerOccupancy();
//     return true;
// }

bool ControlScan::show(const std::string &file, bool pub_octo)
{
    hirop_msgs::LoadPCL srv;
    srv.request.fileName = file;
    srv.request.isOctoMap = pub_octo;
    load_pcl_client.call(srv);
    return !srv.response.result;

    // pcl::PointCloud<pcl::PointXYZRGB> cloud;
    // sensor_msgs::PointCloud2 output;
    // pcl::io::loadPCDFile(file, cloud); //修改自己pcd文件所在路径
    // pcl::PCLPointCloud2 pcd_tmp;
    // pcl::toPCLPointCloud2(cloud, pcd_tmp);
    // pcl_conversions::fromPCL(pcd_tmp, output);
    // output.header.frame_id = "world";
    // pcl_pub.publish(output);

    // if (pub_octo)
    // {
    //     octomap::ColorOcTree tree(0.05);
    //     if (!PointConvertOctompRBG(tree, cloud))
    //         return false;

    //     static octomap_msgs::Octomap octomap;
    //     octomap_msgs::binaryMapToMsg(tree, octomap);

    //     moveit_msgs::PlanningScene planning_scene;
    //     planning_scene.world.octomap.header.stamp = ros::Time::now();
    //     planning_scene.world.octomap.header.frame_id = "world";
    //     planning_scene.world.octomap.octomap.header.stamp = ros::Time::now();
    //     planning_scene.world.octomap.octomap.header.frame_id = "world";
    //     planning_scene.world.octomap.octomap.binary = true;
    //     planning_scene.world.octomap.octomap.id = "OcTree";
    //     planning_scene.world.octomap.octomap.data = octomap.data;
    //     planning_scene.world.octomap.octomap.resolution = 0.05;
    //     planning_scene.is_diff = true;
    //     publishOctomapPub.publish(planning_scene);
    // }
    // ros::Duration(0.5).sleep();
    // return true;
}

const geometry_msgs::PoseStamped &ControlScan::getBeginPose() const
{
    return begin_pose;
}

template <typename T>
bool ControlScan::callServer(ros::ServiceClient &client, T &srv)
{
    bool flag;
    flag = client.call(srv);
    ROS_INFO_STREAM(client.getService() << ": " << (flag ? "SUCCESS" : "FAILED"));
    return flag;
}