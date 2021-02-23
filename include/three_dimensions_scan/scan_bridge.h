#pragma once

#include "scan_framework.h"
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

#include "config.h"

#ifdef TEST
#include "three_dimensions_scan/AutoScan.h"
#include "three_dimensions_scan/ScanFile.h"
#include "three_dimensions_scan/InsertPose.h"
#include "three_dimensions_scan/PubScene.h"
#define MSGS_FILE three_dimensions_scan
#else
#include "hirop_msgs/AutoScan.h"
#define MSGS_FILE hirop_msgs
#endif

class ScanBridge
{
public:
    ScanBridge(ros::NodeHandle *n);
    ~ScanBridge();

private:
    // bool hmScanCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &rep);
    bool autoScanCB(MSGS_FILE::AutoScan::Request &req, MSGS_FILE::AutoScan::Response &rep);
    bool rotateCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep);
    bool addPoseCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep);
    bool savePoseCB(MSGS_FILE::ScanFile::Request &req, MSGS_FILE::ScanFile::Response &rep);
    bool loadPoseCB(MSGS_FILE::ScanFile::Request &req, MSGS_FILE::ScanFile::Response &rep);
    bool insertPoseCB(MSGS_FILE::InsertPose::Request &req, MSGS_FILE::InsertPose::Response &rep);
    bool motionRecordPoseCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep);
    bool saveScanDataCB(MSGS_FILE::ScanFile::Request &req, MSGS_FILE::ScanFile::Response &rep);
    bool loadScanDataCB(MSGS_FILE::PubScene::Request &req, MSGS_FILE::PubScene::Response &rep);
    bool resetScanCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep);
    bool rmWorkspaceCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep);
    bool stopMotionCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep);
    bool pauseScanCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep);
    bool resumeScanCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep);

private:
    ros::NodeHandle *nh_;

    std::shared_ptr<ScanFramework> sf_ptr;

    // ros::ServiceServer hm_scan_server;
    ros::ServiceServer auto_scan_server;
    ros::ServiceServer rotato_scan_server;
    ros::ServiceServer add_pose_server;
    ros::ServiceServer save_pose_server;
    ros::ServiceServer load_pose_server;
    ros::ServiceServer insert_pose_server;
    ros::ServiceServer motion_record_pose_server;
    ros::ServiceServer save_scan_data;
    ros::ServiceServer load_scan_data;
    ros::ServiceServer reset_scan_data;
    ros::ServiceServer rm_workspace_server;
    ros::ServiceServer stop_motion_server;
    ros::ServiceServer pause_scan_server;
    ros::ServiceServer resume_scan_server;
};
