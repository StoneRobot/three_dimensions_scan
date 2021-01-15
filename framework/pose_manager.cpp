#include "pose_manager.h"

PoseManager::PoseManager(ros::NodeHandle *n)
    : nh_{n},
      file_uri{"three_dimensions"},
      position_step{0.04},
      rpy_step{0.157},
      travel{M_PI / 2}
{
    load_pose_client = nh_->serviceClient<hirop_msgs::loadJointsData>("load_joint_data");
    record_joint_pose_client = nh_->serviceClient<hirop_msgs::saveJointData>("add_joint_data");
    save_pose_client = nh_->serviceClient<hirop_msgs::saveDataEnd>("save_data_end");

    record_cartesian_pose_client = nh_->serviceClient<hirop_msgs::savePoseData>("add_pose_data");
    load_cartesian_pose_client = nh_->serviceClient<hirop_msgs::loadPoseData>("load_pose_data");
}

PoseManager::~PoseManager()
{
}

void PoseManager::generatorPose(const LookingParam &lp, std::vector<geometry_msgs::PoseStamped> &out_poses)
{
    orientation.setRPY(lp.R, lp.P, lp.Y);
    generateOrientation(lp, out_poses);
    generatePosition(lp, out_poses);
}

void PoseManager::generatePosition(const LookingParam &lp, std::vector<geometry_msgs::PoseStamped> &pose_list)
{
    geometry_msgs::PoseStamped p;
    double x = lp.radius;
    int cnt = lp.radius / position_step;
    double y1, y2;
    for (int i = 0; i < cnt; i++)
    {
        getYPosition(lp.radius, x, y1, y2);
        p.header.frame_id = lp.frame_id;
        p.pose.position.x = x;
        p.pose.position.y = y1;
        p.pose.position.z = lp.h;
        p.pose.orientation = tf2::toMsg(orientation);
        pose_list.push_back(p);
        p.pose.position.y = y2;
        pose_list.push_back(p);
        x -= position_step;
    }
}

void PoseManager::generateOrientation(const LookingParam &lp, std::vector<geometry_msgs::PoseStamped> &pose)
{
    tf2::Quaternion ori;
    if (!lp.fixed_R)
        tfQuatRotate(lp, 1, 0, 0, pose);
    if (!lp.fixed_P)
        tfQuatRotate(lp, 0, 1, 0, pose);
    if (!lp.fixed_Y)
        tfQuatRotate(lp, 0, 0, 1, pose);
}

void PoseManager::tfQuatRotate(const LookingParam &lp, int vx, int vy, int vz, std::vector<geometry_msgs::PoseStamped> &pose)
{
    tf2::Quaternion orien;
    int cnt = travel / rpy_step;
    int cnt_half = -(cnt / 2);
    for (cnt_half; cnt_half < cnt / 2; ++cnt_half)
    {
        tf2::Quaternion increment;
        geometry_msgs::PoseStamped p;
        increment.setRPY(vx * rpy_step * cnt_half, vy * rpy_step * cnt_half, vz * rpy_step * cnt_half);
        orien = orientation * increment;
        p.pose.orientation = tf2::toMsg(orien);
        p.header.frame_id = lp.frame_id;
        p.pose.position.x = lp.radius;
        p.pose.position.y = 0;
        p.pose.position.z = lp.h;
        pose.push_back(p);
    }
}

void PoseManager::getYPosition(const double &r, const double &x, double &y1, double &y2)
{
    double Y = sqrt((pow(r, 2) - pow(x, 2)));
    y1 = Y;
    y2 = -Y;
}

bool PoseManager::loadPose(const std::string &file, std::vector<std::vector<double>>& joints)
{
    hirop_msgs::loadJointsData srv;
    srv.request.uri = file_uri;
    srv.request.name = file;
    if (load_pose_client.call(srv))
    {
        int count = srv.response.joints.size();
        joints.resize(count);
        for (int i = 0; i < count; ++i)
        {
            joints[i] = srv.response.joints[i].joint;
        }
        return true;
    }
    return false;
}

bool PoseManager::loadPose(const std::string &file, std::vector<geometry_msgs::PoseStamped> &pose)
{
    hirop_msgs::loadPoseData srv;
    srv.request.uri = file_uri;
    srv.request.name = file;
    if(load_cartesian_pose_client.call(srv))
    {
        pose = srv.response.poses;
        return true;
    }
    return false;
}

bool PoseManager::recordPose(const std::vector<double> &joint)
{
    hirop_msgs::saveJointData srv;
    srv.request.joint = joint;
    if (record_joint_pose_client.call(srv))
    {
        if (srv.response.result == 0)
            return true;
    }
    return false;
}

bool PoseManager::recordPose(const geometry_msgs::PoseStamped& pose)
{
    hirop_msgs::savePoseData srv;
    srv.request.pose = pose;
    if (record_cartesian_pose_client.call(srv))
    {
        if (srv.response.result == 0)
            return true;
    }
    return false;
}

bool PoseManager::savePose(const std::string &file)
{
    hirop_msgs::saveDataEnd srv;
    srv.request.uri = file_uri;
    srv.request.name = file;
    if (save_pose_client.call(srv))
    {
        if (srv.response.result == 0)
            return true;
    }
    return false;
}
