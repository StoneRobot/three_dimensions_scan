#pragma once

#include "scan_framework.h"
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

#include "config.h"

#ifdef TEST
#include "three_dimensions_scan/AutoScan.h"
#include "three_dimensions_scan/ScanFile.h"
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
    bool hmScanCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &rep);
    bool autoScanCB(MSGS_FILE::AutoScan::Request &req, MSGS_FILE::AutoScan::Response &rep);
    bool rotateCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep);
    bool addPoseCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep);
    bool savePoseCB(MSGS_FILE::ScanFile::Request &req, MSGS_FILE::ScanFile::Response &rep);
    bool motionRecordPoseCB(MSGS_FILE::ScanFile::Request &req, MSGS_FILE::ScanFile::Response &rep);
    bool saveScanDataCB(MSGS_FILE::ScanFile::Request &req, MSGS_FILE::ScanFile::Response &rep);

private:
    ros::NodeHandle *nh_;

    std::shared_ptr<ScanFramework> sf_ptr;

    ros::ServiceServer hm_scan_server;
    ros::ServiceServer auto_scan_server;
    ros::ServiceServer rotato_scan_server;
    ros::ServiceServer add_pose_server;
    ros::ServiceServer save_pose_server;
    ros::ServiceServer motion_record_pose_server;
    ros::ServiceServer save_scan_data;
};
