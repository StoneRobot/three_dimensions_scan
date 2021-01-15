#include "motion_actuator.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spiner(4);
    spiner.start();
    moveit::planning_interface::MoveGroupInterface* mg = new moveit::planning_interface::MoveGroupInterface("arm");
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg_ptr(mg);
    MotionActuator*  ma = new MotionActuator(&nh, mg);
    std::shared_ptr<MotionActuator> ma_ptr(ma);

    std::vector<std::vector<double> > joints;
    joints.resize(2);
    mg->setStartStateToCurrentState();
    joints[0] = mg->getCurrentJointValues();
    joints[0][0] += 0.1;
    joints[1] = joints[0];
    joints[1][0] -= 0.5;
    std::cout << ma->motionRecordPose(joints) << std::endl;

    std::vector<geometry_msgs::PoseStamped> poses;
    poses.resize(2);
    poses[0].header.frame_id = "world";
    poses[0].pose.orientation.w = 1;
    poses[0].pose.position.x = 0.85;
    poses[0].pose.position.y = 0;
    poses[0].pose.position.z = 1.3;
    poses[1] = poses[0];
    poses[1].pose.position.y = 0.1;
    ma->setPose(poses);
    ma->autoMotion();
    return 0;
}
