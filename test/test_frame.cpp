#include "scan_framework.h"

using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spiner(4);
    spiner.start();

    shared_ptr<ScanFramework> scan_fw_ptr = make_shared<ScanFramework>();

    LookingParam lp;
    lp.frame_id = "world";
    lp.h = 1.4;
    lp.radius = 0.7;
    lp.fixed_R = true;
    lp.R = 0;
    lp.fixed_P = false;
    lp.P = 0;
    lp.fixed_Y = false;
    lp.Y = 0;
    scan_fw_ptr->autoScan(lp);

    scan_fw_ptr->RecordPose();
    scan_fw_ptr->autoScan(lp);
    scan_fw_ptr->RecordPose();
    scan_fw_ptr->savePose("test");

    scan_fw_ptr->motionRcordPose("test");
    scan_fw_ptr->rotate();
    return 0;
}
