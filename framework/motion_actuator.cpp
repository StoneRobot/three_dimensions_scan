#include "motion_actuator.h"

using namespace std;

MotionActuator::MotionActuator(ros::NodeHandle *n, moveit::planning_interface::MoveGroupInterface *move_group)
    : nh_{n},
      load_obj_client{nh_->serviceClient<hirop_msgs::PubObject>("/loadObject")},
      move_group_{move_group},
      plan_count{1},
      execute_count{1},
      joint_1_bounds{move_group_->getRobotModel()->getVariableBounds(move_group_->getJointNames()[0])}
{
    setConstraint(true, false);
    move_group_->setMaxVelocityScalingFactor(1);
    move_group_->setMaxAccelerationScalingFactor(1);
}

MotionActuator::~MotionActuator()
{
    ROS_INFO_STREAM("<<<<delete MotionActoator>>>");
}

void MotionActuator::setPose(const std::vector<double> &joint)
{
    move_group_->setJointValueTarget(joint);
}

void MotionActuator::setPose(const std::vector<geometry_msgs::PoseStamped> &poses)
{
    move_group_->setPoseTargets(poses, ws_.tcp);
}

bool MotionActuator::autoMotion()
{
    bool flag = false;
    setConstraint(false, false);
    if (planAndMove())
        flag = rotate();
    return flag;
}

bool MotionActuator::rotate()
{
    setConstraint(false, true);
    move_group_->setPlanningTime(10);
    vector<double> joints = move_group_->getCurrentJointValues();
    joints[0] = joint_1_bounds.min_position_ + 0.1;
    setPose(joints);
    planAndMove();
    joints[0] = joint_1_bounds.max_position_ - 0.1;
    setPose(joints);
    return planAndMove();
}

double MotionActuator::motionRecordPose(const std::vector<std::vector<double>> &joints)
{
    int count = joints.size();
    int cnt = 0;
    for (auto i : joints)
    {
        setPose(i);
        if (autoMotion())
            ++cnt;
    }
    return static_cast<double>(cnt / count);
}

const vector<double> MotionActuator::getJointPose() const
{
    move_group_->setStartStateToCurrentState();
    return move_group_->getCurrentJointValues();
}

const geometry_msgs::PoseStamped MotionActuator::getPose() const
{
    move_group_->setStartStateToCurrentState();
    return move_group_->getCurrentPose();
}

bool MotionActuator::plan()
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    int plan_cnt = 0;
    bool flag = false;
    move_group_->setStartStateToCurrentState();
    while (ros::ok() && plan_cnt < plan_count)
    {
        if (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            flag = true;
        }
        ++plan_cnt;
    }
    return flag;
}

bool MotionActuator::move()
{
    int exe_cnt = 0;
    bool flag = false;
    while (ros::ok() && exe_cnt < execute_count)
    {
        if (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            flag = true;
        }
        ++exe_cnt;
    }
    return flag;
}

bool MotionActuator::planAndMove()
{
    bool flag = false;
    if (plan())
        flag = move();
    return flag;
}

void MotionActuator::setConstraint(const bool &is_set_workspace, const bool &is_set_joint_con)
{
    move_group_->clearPathConstraints();
    moveit_msgs::Constraints con;
    if (is_set_workspace)
        getWorkspace(con);
    if (is_set_joint_con)
        getJointConstraint(con);
    move_group_->setPathConstraints(con);
}

// TODO 转动时设置了无法动作
void MotionActuator::getWorkspace(moveit_msgs::Constraints &con)
{
    nh_->param("/workspace/max_x", ws_.max_x, double(1));
    nh_->param("/workspace/min_x", ws_.min_x, double(-1));
    nh_->param("/workspace/max_y", ws_.max_y, double(1));
    nh_->param("/workspace/min_y", ws_.min_y, double(-1));
    nh_->param("/workspace/max_z", ws_.max_z, double(2));
    nh_->param("/workspace/min_z", ws_.min_z, double(1));
    nh_->param("/workapace/frame_id", ws_.frame_id, string("world"));
    nh_->param("/workspace/eef", ws_.tcp, string(move_group_->getEndEffectorLink()));
    ROS_INFO("max_x: %lf, min_x: %lf, max_y: %lf, min_y: %lf, max_z: %lf, min_z: %lf, frame_id: %s, tcp: %s",
             ws_.max_x, ws_.min_x, ws_.max_y, ws_.min_y, ws_.max_z, ws_.min_z, ws_.frame_id.c_str(), ws_.tcp.c_str());
    // con.position_constraints.resize(1);
    // con.position_constraints[0].header.frame_id = ws_.frame_id;
    // con.position_constraints[0].constraint_region.primitives.resize(1);
    // con.position_constraints[0].constraint_region.primitives[0].type = con.position_constraints[0].constraint_region.primitives[0].BOX;
    // con.position_constraints[0].constraint_region.primitives[0].dimensions.resize(3);
    // con.position_constraints[0].constraint_region.primitives[0].dimensions[0] = ws_.max_x - ws_.min_x;
    // con.position_constraints[0].constraint_region.primitives[0].dimensions[1] = ws_.max_y - ws_.min_y;
    // con.position_constraints[0].constraint_region.primitives[0].dimensions[2] = ws_.max_z - ws_.min_z;
    // con.position_constraints[0].constraint_region.primitive_poses.resize(1);
    // con.position_constraints[0].constraint_region.primitive_poses[0].position.x = (ws_.max_x + ws_.min_x)/2;
    // con.position_constraints[0].constraint_region.primitive_poses[0].position.y = (ws_.max_y + ws_.min_y)/2;
    // con.position_constraints[0].constraint_region.primitive_poses[0].position.z = (ws_.max_z + ws_.min_z)/2;
    // con.position_constraints[0].constraint_region.primitive_poses[0].orientation.x = 0;
    // con.position_constraints[0].constraint_region.primitive_poses[0].orientation.y = 0;
    // con.position_constraints[0].constraint_region.primitive_poses[0].orientation.z = 0;
    // con.position_constraints[0].constraint_region.primitive_poses[0].orientation.w = 1;
    // con.position_constraints[0].link_name = ws_.tcp;
    // con.position_constraints[0].target_point_offset.x = 0.02;
    // con.position_constraints[0].target_point_offset.y = 0.02;
    // con.position_constraints[0].target_point_offset.z = 0.02;
    // con.position_constraints[0].weight = 1;
    vector<string> ob_name = {"q", "h", "z", "y", "s", "x"};
    hirop_msgs::PubObject srv;
    srv.request.header.frame_id = ws_.frame_id;
    // srv.request.object_id.resize(6);
    srv.request.object_id = ob_name;
    srv.request.pose.resize(6);
    srv.request.primitives.resize(6);
    srv.request.rgba.resize(6);
    for (int i = 0; i < 6; i++)
    {
        srv.request.rgba[i].a = 0.25;
        srv.request.pose[i].position.x = (ws_.max_x + ws_.min_x) / 2;
        srv.request.pose[i].position.y = (ws_.max_y + ws_.min_y) / 2;
        srv.request.pose[i].position.z = (ws_.max_z + ws_.min_z) / 2;

        srv.request.primitives[i].type = srv.request.primitives[i].BOX;
        srv.request.primitives[i].dimensions.resize(3);
        srv.request.primitives[i].dimensions[0] = ws_.max_x - ws_.min_x;
        srv.request.primitives[i].dimensions[1] = ws_.max_y - ws_.min_y;
        srv.request.primitives[i].dimensions[2] = ws_.max_z - ws_.min_z;
        if (i == 0)
        {
            srv.request.pose[i].position.x = ws_.max_x;
            srv.request.primitives[i].dimensions[0] = 0.001;
            continue;
        }
        else if (i == 1)
        {
            srv.request.primitives[i].dimensions[0] = 0.001;
            srv.request.pose[i].position.x = ws_.min_x;
            continue;
        }
        else if (i == 2)
        {
            srv.request.primitives[i].dimensions[1] = 0.001;
            srv.request.pose[i].position.y = ws_.min_y;
            continue;
        }
        else if (i == 3)
        {
            srv.request.primitives[i].dimensions[1] = 0.001;
            srv.request.pose[i].position.y = ws_.max_y;
            continue;
        }
        else if (i == 4)
        {
            srv.request.primitives[i].dimensions[2] = 0.001;
            srv.request.pose[i].position.z = ws_.max_z;
            continue;
        }
        else if (i == 5)
        {
            srv.request.primitives[i].dimensions[2] = 0.001;
            srv.request.pose[i].position.z = ws_.min_z;
            continue;
        }
    }
    load_obj_client.call(srv);
}

void MotionActuator::getJointConstraint(moveit_msgs::Constraints &con)
{
    con.joint_constraints.resize(5);
    move_group_->setStartStateToCurrentState();
    vector<string> joint_name = move_group_->getJointNames();
    vector<double> joint_value = move_group_->getCurrentJointValues();
    for (int i = 0; i < 5; i++)
    {
        con.joint_constraints[i].joint_name = joint_name[i + 1];
        con.joint_constraints[i].position = joint_value[i + 1];
        con.joint_constraints[i].tolerance_above = 0.1;
        con.joint_constraints[i].tolerance_below = 0.1;
        con.joint_constraints[i].weight = 1;
    }
}
