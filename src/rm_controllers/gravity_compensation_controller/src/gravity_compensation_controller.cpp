#include "gravity_compensation_controller/gravity_compensation_controller.h"
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <memory>
#include "XmlRpcValue.h"
#include "kdl_parser/kdl_parser.hpp"

namespace gravity_compensation_controller
{
bool GravityCompensationController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
{
  if (!JointTrajectoryController::init(hw, root_nh, controller_nh))
  {
    return false;
  }

  XmlRpc::XmlRpcValue offset_torques;
  controller_nh.param("offset_torques", offset_torques, {0.0});
  if (offset_torques.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Failed to get offset torques, unexpected type.");
    return false;
  }
  for (int i = 0; i < offset_torques.size(); ++i)
  {
    if (offset_torques[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      ROS_ERROR("Failed to get offset torques, unexpected type in array.");
      return false;
    }
    offset_torques_.push_back(static_cast<double>(offset_torques[i]));
  }

  if (!model_.initParam("robot_description"))
  {
    ROS_ERROR("Failed to get robot description.");
    return false;
  }

  if (!kdl_parser::treeFromUrdfModel(model_, tree_))
  {
    ROS_ERROR("Failed to parse KDL tree from URDF.");
    return false;
  }

  controller_nh.param("chain_root", chain_root_, std::string("base_link"));
  controller_nh.param("chain_tip", chain_tip_, std::string("gripper_link"));

  if (!tree_.getChain(chain_root_, chain_tip_, chain_))
  {
    ROS_ERROR("Failed to get KDL chain from tree.");
    return false;
  }

  joint_positions_.resize(chain_.getNrOfJoints());

  KDL::Vector gravity{0.0, 0.0, -9.81};
  solver_ = std::make_unique<KDL::ChainDynParam>(chain_, gravity);

  return true;
}

void GravityCompensationController::update(const ros::Time& time, const ros::Duration& period)
{
  // Get currently followed trajectory
  TrajectoryPtr curr_traj_ptr;
  curr_trajectory_box_.get(curr_traj_ptr);
  Trajectory& curr_traj = *curr_traj_ptr;

  old_time_data_ = *(time_data_.readFromRT());

  // Update time data
  TimeData time_data;
  time_data.time   = time;                                     // Cache current time
  time_data.period = period;                                   // Cache current control period
  time_data.uptime = old_time_data_.uptime + period; // Update controller uptime
  time_data_.writeFromNonRT(time_data); // TODO: Grrr, we need a lock-free data structure here!

  // NOTE: It is very important to execute the two above code blocks in the specified sequence: first get current
  // trajectory, then update time data. Hopefully the following paragraph sheds a bit of light on the rationale.
  // The non-rt thread responsible for processing new commands enqueues trajectories that can start at the _next_
  // control cycle (eg. zero start time) or later (eg. when we explicitly request a start time in the future).
  // If we reverse the order of the two blocks above, and update the time data first; it's possible that by the time we
  // fetch the currently followed trajectory, it has been updated by the non-rt thread with something that starts in the
  // next control cycle, leaving the current cycle without a valid trajectory.

  updateStates(time_data.uptime, curr_traj_ptr.get());

  // Update current state and state error
  for (unsigned int i = 0; i < getNumberOfJoints(); ++i)
  {
    typename TrajectoryPerJoint::const_iterator segment_it = sample(curr_traj[i], time_data.uptime.toSec(), desired_joint_state_);
    if (curr_traj[i].end() == segment_it)
    {
      // Non-realtime safe, but should never happen under normal operation
      ROS_ERROR_NAMED(name_,
                      "Unexpected error: No trajectory defined at current time. Please contact the package maintainer.");
      return;
    }

    // Get state error for current joint
    state_joint_error_.position[0] = state_error_.position[i];
    state_joint_error_.velocity[0] = state_error_.velocity[i];
    state_joint_error_.acceleration[0] = state_error_.acceleration[i];

    //Check tolerances
    const RealtimeGoalHandlePtr rt_segment_goal = segment_it->getGoalHandle();
    if (rt_segment_goal && rt_segment_goal == rt_active_goal_)
    {
      // Check tolerances
      if (time_data.uptime.toSec() < segment_it->endTime())
      {
        // Currently executing a segment: check path tolerances
        const joint_trajectory_controller::SegmentTolerancesPerJoint<Scalar>& joint_tolerances = segment_it->getTolerances();
        if (!checkStateTolerancePerJoint(state_joint_error_, joint_tolerances.state_tolerance))
        {
          if (verbose_)
          {
            ROS_ERROR_STREAM_NAMED(name_,"Path tolerances failed for joint: " << joint_names_[i]);
            checkStateTolerancePerJoint(state_joint_error_, joint_tolerances.state_tolerance, true);
          }
          rt_segment_goal->preallocated_result_->error_code =
          control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
          rt_segment_goal->preallocated_result_->error_string = joint_names_[i] + " path error " + std::to_string( state_joint_error_.position[0] );
          rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
          // Force this to run before destroying rt_active_goal_ so results message is returned
          rt_active_goal_->runNonRealtime(ros::TimerEvent());
          rt_active_goal_.reset();
          successful_joint_traj_.reset();
        }
      }
      else if (segment_it == --curr_traj[i].end())
      {
        if (verbose_)
          ROS_DEBUG_STREAM_THROTTLE_NAMED(1,name_,"Finished executing last segment, checking goal tolerances");

        // Controller uptime
        const ros::Time uptime = time_data_.readFromRT()->uptime;

        // Checks that we have ended inside the goal tolerances
        const joint_trajectory_controller::SegmentTolerancesPerJoint<Scalar>& tolerances = segment_it->getTolerances();
        const bool inside_goal_tolerances = checkStateTolerancePerJoint(state_joint_error_, tolerances.goal_state_tolerance);

        if (inside_goal_tolerances)
        {
          successful_joint_traj_[i] = 1;
        }
        else if (uptime.toSec() < segment_it->endTime() + tolerances.goal_time_tolerance)
        {
          // Still have some time left to meet the goal state tolerances
        }
        else
        {
          if (verbose_)
          {
            ROS_ERROR_STREAM_NAMED(name_,"Goal tolerances failed for joint: "<< joint_names_[i]);
            // Check the tolerances one more time to output the errors that occurs
            checkStateTolerancePerJoint(state_joint_error_, tolerances.goal_state_tolerance, true);
          }

          rt_segment_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
          rt_segment_goal->preallocated_result_->error_string = joint_names_[i] + " goal error " + std::to_string( state_joint_error_.position[0] );
          rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
          // Force this to run before destroying rt_active_goal_ so results message is returned
          rt_active_goal_->runNonRealtime(ros::TimerEvent());
          rt_active_goal_.reset();
          successful_joint_traj_.reset();
        }
      }
    }
  }

  //If there is an active goal and all segments finished successfully then set goal as succeeded
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
  if (current_active_goal && successful_joint_traj_.count() == getNumberOfJoints())
  {
    current_active_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
    current_active_goal.reset(); // do not publish feedback
    rt_active_goal_.reset();
    successful_joint_traj_.reset();
  }

  updateFuncExtensionPoint(curr_traj, time_data);

  // Hardware interface adapter: Generate and send commands
  hw_iface_adapter_.updateCommand(time_data.uptime, time_data.period,
                                  desired_state_, state_error_);

  joint_trajectory_controller::setActionFeedback();

  publishState(time_data.uptime);
}

void GravityCompensationController::computeGravityTorques()
{
  int i = 0;
  for (auto& joint : joints_)
  {
    joint_positions_(i) = joint.getPosition();
    ++i;
  }
  KDL::JntArray tau_g;
  solver_->JntToGravity(joint_positions_, tau_g);
  gravity_torques_.resize(tau_g.rows());

  for (int j = 0; j < tau_g.rows(); ++j)
  {
    gravity_torques_[j] = tau_g(j) + (offset_torques_.size() > j ? offset_torques_[j] : 0.0);
  }
}
} // namespace gravity_compensation_controller