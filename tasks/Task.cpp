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

Eigen::Vector3d Task::quat2eul(Eigen::Quaterniond q)
{
    Eigen::Vector3d eul;
    double t2 = q.w() * q.w();
    double t3 = q.x() * q.x();
    double t4 = q.y() * q.y();
    double t5 = q.z() * q.z();
    double t6 = -t4;
    eul << atan2(q.w() * q.z() * 2.0 + q.x() * q.y() * 2.0, t2 + t3 - t5 + t6),
        asin(q.w() * q.y() * 2.0 - q.x() * q.z() * 2.0),
        atan2(q.w() * q.x() * 2.0 + q.y() * q.z() * 2.0, t2 - t3 + t5 + t6);
    return eul;
}

double Task::angDiff(double d){
    return std::fmod(d + M_PI, 2.0*M_PI) - M_PI;
}

void Task::groundtruth_poseCallback(const base::Time &ts, const ::base::samples::RigidBodyState &groundtruth_pose_sample)
{
    if (groundtruth_pose_sample.hasValidPosition() and groundtruth_pose_sample.hasValidOrientation())
        gt_pose=groundtruth_pose_sample;
}

void Task::odometry_poseCallback(const base::Time &ts, const ::base::samples::RigidBodyState &odometry_pose_sample)
    {
    count_odo++;
    if (!gt_initial and gt_pose.hasValidPosition() and gt_pose.hasValidOrientation() and count_odo>_skip_first_n)
    {
        gt_initial = true;
        gt_initial_pose = gt_pose;
        odo_initial_pose_inverse=odometry_pose_sample.getTransform().inverse();
        gt_previous_pose = gt_pose;

        // initialize accum pose to 0
        accum_distance = 0;
    }
    if (gt_initial){
        base::samples::RigidBodyState odo_pose;
        odo_pose.setTransform(odo_initial_pose_inverse*odometry_pose_sample.getTransform());
        odo_pose.time=odometry_pose_sample.time;
        // add initial pose to viso pose
        //odo_in_world_pose.position[0]= (cos(gt_initial_pose.getYaw())*odo_pose.position[0] - sin(gt_initial_pose.getYaw())*odo_pose.position[1] + gt_initial_pose.position[0]);
        //odo_in_world_pose.position[1]= (sin(gt_initial_pose.getYaw())*odo_pose.position[0] + cos(gt_initial_pose.getYaw())*odo_pose.position[1] + gt_initial_pose.position[1]);
        //odo_in_world_pose.position[2]= odo_pose.position[2] + gt_initial_pose.position[2];
        odo_in_world_pose.setTransform(gt_initial_pose.getTransform()*odo_pose.getTransform());
        // add initial orientation to viso orientation
        Eigen::Quaternion <double> attitude(Eigen::AngleAxisd(gt_initial_pose.getYaw()+odo_pose.getYaw(), Eigen::Vector3d::UnitZ())*
                                            Eigen::AngleAxisd(odo_pose.getPitch(), Eigen::Vector3d::UnitY()) *
                                            Eigen::AngleAxisd(odo_pose.getRoll(), Eigen::Vector3d::UnitX()));
        odo_in_world_pose.orientation = attitude;
        // add time to the output pose
        odo_in_world_pose.time = odo_pose.time;
        // write out pose
        _odometry_in_world_pose.write(odo_in_world_pose);

        // compute diff pose
        diff_pose.position = gt_pose.position - odo_in_world_pose.position;
        // compute diff orientation
        Eigen::Quaternion <double> attitude2(Eigen::AngleAxisd(gt_pose.getYaw()-odo_in_world_pose.getYaw(), Eigen::Vector3d::UnitZ())*
                                            Eigen::AngleAxisd(gt_pose.getPitch()-odo_in_world_pose.getPitch(), Eigen::Vector3d::UnitY()) *
                                            Eigen::AngleAxisd(gt_pose.getRoll()-odo_in_world_pose.getRoll(), Eigen::Vector3d::UnitX()));
        diff_pose.orientation = attitude2;
        // add time to the output diff pose
        diff_pose.time = odo_pose.time;
        // write out the diff pose
        _diff_pose.write(diff_pose);

        // output gt pose from vicon
        _ground_truth_pose.write(gt_pose);
        // output gt heading from vicon
        _ground_truth_heading.write(gt_pose.getYaw());
        // output viso heading
        _odometry_heading.write(odo_in_world_pose.getYaw());
        _odometry_roll.write(odo_in_world_pose.getRoll());
        _odometry_pitch.write(odo_in_world_pose.getPitch());

        // compute travelled distance
        delta_gt_pose.position[0] = gt_pose.position[0] - gt_previous_pose.position[0];
        delta_gt_pose.position[1] = gt_pose.position[1] - gt_previous_pose.position[1];
        delta_gt_pose.position[2] = gt_pose.position[2] - gt_previous_pose.position[2];
        accum_distance += sqrt(pow(delta_gt_pose.position[0],2)+
                               pow(delta_gt_pose.position[1],2)+
                               pow(delta_gt_pose.position[2],2));
        //output the travelled distance
        _travelled_distance.write(accum_distance);

        // compute percentage error
        error_norm = sqrt(pow(diff_pose.position[0],2)+
                          pow(diff_pose.position[1],2)+
                          pow(diff_pose.position[2],2));
        
        Eigen::Vector3d diff_eul=quat2eul(attitude2);
        double heading_error = sqrt(pow(angDiff(diff_eul[0]),2))*180.0/M_PI;
        _position_error.write(error_norm);
        _heading_error.write(heading_error);

        error_perc = error_norm/accum_distance;
        // output percentage error
        _perc_error.write(error_perc);

        // update previous gt pose
        gt_previous_pose.position[0] = gt_pose.position[0];
        gt_previous_pose.position[1] = gt_pose.position[1];
        gt_previous_pose.position[2] = gt_pose.position[2];

        if (!_align_streams){
            double lag=(gt_pose.time-odo_pose.time).toSeconds();
            _lag.write(lag);
        }
    }
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
    count_odo=0;
    gt_initial=false;
    gt_pose.invalidate();
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    if (!_align_streams){
        base::samples::RigidBodyState gt_pose_new;
        if (_groundtruth_pose.read(gt_pose_new) == RTT::NewData)
        {
            groundtruth_poseCallback(gt_pose_new.time, gt_pose_new);
        }    
        base::samples::RigidBodyState odo_pose_new;
        if (_odometry_pose.read(odo_pose_new) == RTT::NewData)
        {
            odometry_poseCallback(odo_pose_new.time, odo_pose_new);
        }
    }
    if (_reset.read(reset) == RTT::NewData)
    {
        count_odo=0;
        gt_initial = false;
        gt_pose.invalidate();
    }
    TaskBase::updateHook();
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
