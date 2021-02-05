#include "three_dimensions_scan/scan_bridge.h"

using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "three_dimensions_scan");
    ros::NodeHandle nh;
    ros::AsyncSpinner spiner(4);
    spiner.start();

    ScanBridge s(&nh);
    ros::waitForShutdown();
    return 0;
}