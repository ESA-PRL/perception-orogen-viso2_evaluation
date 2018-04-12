/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace viso2_evaluation;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    if (_reset.read(reset) == RTT::NewData)
    {
        gt_initial = false;
    }

    if (_groundtruth_pose.read(gt_pose) == RTT::NewData)
    {
        if (!gt_initial)
        {
            gt_initial = true;
            gt_initial_pose = gt_pose;
        }
    }

    if (_odometry_pose.read(odo_pose) == RTT::NewData && gt_initial)
    {
        odo_in_world_pose.position[0]= (cos(gt_initial_pose.getYaw())*odo_pose.position[0] - sin(gt_initial_pose.getYaw())*odo_pose.position[1] + gt_initial_pose.position[0]);
        odo_in_world_pose.position[1]= (sin(gt_initial_pose.getYaw())*odo_pose.position[0] + cos(gt_initial_pose.getYaw())*odo_pose.position[1] + gt_initial_pose.position[1]);
        odo_in_world_pose.position[2]= odo_pose.position[2] + gt_initial_pose.position[2];
        Eigen::Quaternion <double> attitude(Eigen::AngleAxisd(gt_initial_pose.getYaw()+odo_pose.getYaw(), Eigen::Vector3d::UnitZ())*
                                            Eigen::AngleAxisd(odo_pose.getPitch(), Eigen::Vector3d::UnitY()) *
                                            Eigen::AngleAxisd(odo_pose.getRoll(), Eigen::Vector3d::UnitX()));
        odo_in_world_pose.orientation = attitude;
        _odometry_in_world_pose.write(odo_in_world_pose);
        diff_pose.position = gt_pose.position - odo_in_world_pose.position;
        Eigen::Quaternion <double> attitude2(Eigen::AngleAxisd(gt_pose.getYaw()-odo_in_world_pose.getYaw(), Eigen::Vector3d::UnitZ())*
                                            Eigen::AngleAxisd(gt_pose.getPitch()-odo_in_world_pose.getPitch(), Eigen::Vector3d::UnitY()) *
                                            Eigen::AngleAxisd(gt_pose.getRoll()-odo_in_world_pose.getRoll(), Eigen::Vector3d::UnitX()));
        diff_pose.orientation = attitude2;
        _diff_pose.write(diff_pose);
    }
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
