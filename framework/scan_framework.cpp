#include "scan_framework.h"

using namespace std;

ScanFramework::ScanFramework()
    : mg_ptr{make_shared<moveit::planning_interface::MoveGroupInterface>("arm")},
      nh{&const_cast<ros::NodeHandle &>(mg_ptr->getNodeHandle())},
      ma_ptr{make_shared<MotionActuator>(nh, mg_ptr.get())},
      pm_ptr{make_shared<PoseManager>(nh)},
      cs_ptr{make_shared<ControlScan>(nh)}
{
    cs_ptr->bringup();
}

ScanFramework::~ScanFramework()
{
    cs_ptr->close();
}

bool ScanFramework::HMScan(const bool &on_off)
{
    bool flag = false;
    if (on_off)
        flag = cs_ptr->start(ma_ptr->getPose());
    else
        flag = cs_ptr->close();
    return flag;
}

bool ScanFramework::autoScan(const LookingParam &lp)
{
    if (cs_ptr->start(ma_ptr->getPose()))
    {
        vector<geometry_msgs::PoseStamped> poses;
        pm_ptr->generatorPose(lp, poses);
        ma_ptr->setPose(poses);
        return ma_ptr->autoMotion();
    }
    return false;
}

bool ScanFramework::rotate()
{
    return ma_ptr->rotate();
}

bool ScanFramework::loadPose(const std::string &file)
{
    pose_file_name = file;
    motion_joints_pose.clear();
    return pm_ptr->loadPose(file, motion_joints_pose);
}

bool ScanFramework::insertPose(int insert, bool is_save)
{
    vector<double> j = ma_ptr->getJointPose();
    vector<vector<double>>::iterator iter = motion_joints_pose.begin();

    try
    {
        iter += insert;
        motion_joints_pose.insert(iter, j);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }

    if (is_save)
    {
        for (vector<vector<double>>::iterator iter = motion_joints_pose.begin(); iter < motion_joints_pose.end(); iter++)
        {
            if (!pm_ptr->recordPose(*iter))
            {
                return false;
            }
        }
        pm_ptr->savePose(pose_file_name);
    }
    return true;
}

double ScanFramework::motionRcordPose()
{
    double rate;
    vector<double> j;
    j = ma_ptr->getJointPose();
    rate = ma_ptr->motionRecordPose(motion_joints_pose);
    ma_ptr->setPose(j);
    ma_ptr->planAndMove();
    return rate;
}

bool ScanFramework::RecordPose()
{
    return pm_ptr->recordPose(ma_ptr->getJointPose());
}

bool ScanFramework::savePose(const std::string &file)
{
    return pm_ptr->savePose(file);
}

bool ScanFramework::saveScanData(const std::string &file)
{
    bool flag = false;
    string file_pcd;
    geometry_msgs::PoseStamped pose;
    file_pcd = pcd_uri + file;
    ROS_INFO_STREAM("file_pcd: " << file_pcd);
    flag = cs_ptr->save(file_pcd);
    pose = cs_ptr->getBeginPose();
    flag &= pm_ptr->recordPose(pose);
    flag &= pm_ptr->savePose(file);
    return flag;
}

bool ScanFramework::loadScanData(const std::string &file)
{
    bool flag = false;
    string file_pcd;
    vector<geometry_msgs::PoseStamped> p;
    flag = pm_ptr->loadPose(file, p);
    ma_ptr->setPose(p);
    flag &= ma_ptr->planAndMove();
    file_pcd = pcd_uri + file + ".pcd";
    flag &= cs_ptr->show(file_pcd);
    return flag;
}

bool ScanFramework::resetScanData()
{
    return cs_ptr->resetData();
}

bool ScanFramework::rmWorkspace()
{
    return ma_ptr->rmWorkspace();
}

void ScanFramework::stopMotion()
{
    ma_ptr->stopMotion();
}