#include "three_dimensions_scan/scan_bridge.h"

using namespace std;

ScanBridge::ScanBridge(ros::NodeHandle *n)
    : nh_{n},
      sf_ptr{make_shared<ScanFramework>()}
{
    hm_scan_server = nh_->advertiseService("hm_scan", &ScanBridge::hmScanCB, this);
    auto_scan_server = nh_->advertiseService("auto_scan", &ScanBridge::autoScanCB, this);
    rotato_scan_server = nh_->advertiseService("rotate_scan", &ScanBridge::rotateCB, this);
    add_pose_server = nh_->advertiseService("add_scan_pose", &ScanBridge::addPoseCB, this);
    save_pose_server = nh_->advertiseService("save_scan_pose", &ScanBridge::savePoseCB, this);
    motion_record_pose_server = nh_->advertiseService("motion_record_pose", &ScanBridge::motionRecordPoseCB, this);
    save_scan_data = nh_->advertiseService("save_scan_data", &ScanBridge::saveScanDataCB, this);
}

ScanBridge::~ScanBridge()
{
}

bool ScanBridge::hmScanCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &rep)
{
    rep.success = sf_ptr->HMScan(req.data);
    return true;
}

bool ScanBridge::autoScanCB(MSGS_FILE::AutoScan::Request &req, MSGS_FILE::AutoScan::Response &rep)
{
    LookingParam lp;
    lp.frame_id = req.frame_id;
    lp.h = req.h;
    lp.radius = req.radius;
    lp.fixed_R = req.fixed_R;
    lp.R = req.R;
    lp.fixed_P = req.fixed_P;
    lp.P = req.P;
    lp.fixed_Y = req.fixed_Y;
    lp.Y = req.Y;
    rep.result = sf_ptr->autoScan(lp);
    return true;
}

bool ScanBridge::rotateCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep)
{
    sf_ptr->rotate();
    return true;
}

bool ScanBridge::addPoseCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep)
{
    sf_ptr->RecordPose();
    return true;
}

bool ScanBridge::savePoseCB(MSGS_FILE::ScanFile::Request &req, MSGS_FILE::ScanFile::Response &rep)
{
    rep.result = sf_ptr->savePose(req.file);
    return true;
}

bool ScanBridge::motionRecordPoseCB(MSGS_FILE::ScanFile::Request &req, MSGS_FILE::ScanFile::Response &rep)
{
    rep.result = sf_ptr->motionRcordPose(req.file);
    return true;
}

bool ScanBridge::saveScanDataCB(MSGS_FILE::ScanFile::Request &req, MSGS_FILE::ScanFile::Response &rep)
{
    rep.result = sf_ptr->saveScanData(req.file);
    return true;
}