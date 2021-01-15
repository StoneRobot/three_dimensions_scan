#include "pose_manager.h"
#include "motion_actuator.h"

using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spiner(2);
    spiner.start();

    moveit::planning_interface::MoveGroupInterface *mg = new moveit::planning_interface::MoveGroupInterface("arm");
    shared_ptr<moveit::planning_interface::MoveGroupInterface> mg_ptr(mg);

    PoseManager *pm = new PoseManager(&nh);
    shared_ptr<PoseManager> pm_ptr(pm);

    MotionActuator*  ma = new MotionActuator(&nh, mg);
    std::shared_ptr<MotionActuator> ma_ptr(ma);

    mg->setNamedTarget("home");
    mg->move();

    mg->setStartStateToCurrentState();
    vector<double> joint = mg->getCurrentJointValues();
    cout << pm->recordPose(joint) << endl;
    cout << pm->savePose("test") << endl;
    vector<vector<double>> joints;
    pm->loadPose("test", joints);

    LookingParam lp;
    vector<geometry_msgs::PoseStamped> poses;
    lp.frame_id = "world";
    lp.fixed_R = true;
    lp.R = 0;
    lp.fixed_P = false;
    lp.P = 0;
    lp.fixed_Y = false;
    lp.Y = 0;
    lp.h = 1.3;
    lp.radius = 0.40;
    pm->generatorPose(lp, poses);
    ROS_INFO_STREAM(poses.size());
    ma->setPose(poses);
    ma->autoMotion();
    return 0;
}