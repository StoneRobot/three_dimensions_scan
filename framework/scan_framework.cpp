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

double ScanFramework::motionRcordPose(const std::string &file)
{
    vector<vector<double>> joints;
    if (pm_ptr->loadPose(file, joints))
    {
        return ma_ptr->motionRecordPose(joints);
    }
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
    file_pcd = "~/.hirop/data/three_dimensions" + file + ".pcd";
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
    file_pcd = "~/.hirop/data/three_dimensions" + file + ".pcd";
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