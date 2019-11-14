/**
 * @file  open_manipulator_joint_controller.cpp
 * @brief joint controller action server
 */
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>


class JointTrajectoryExecuter
{
private:
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JtActionSrv;
  typedef JtActionSrv::GoalHandle GoalHandle;

  typedef actionlib_msgs::GoalStatus GoalStatus;
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Feedback Feedback;
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Result   Result;

  enum
  {
    NUM_REDUDANT_POSI = 1,
    NUM_GRIPPER_POSI  = 2,
  };

  sensor_msgs::JointState info_;
  uint64_t latest_goal_time_;
  int last_index_;
  std::vector<double> pos_internal_;

  ros::NodeHandle node_;
  JtActionSrv action_server_;
  ros::Publisher pub_joint_state_;
  ros::Publisher pub_arm_down_;
  ros::Subscriber sub_arm_up_;
  std::vector<ros::Publisher> pub_joint_;

  bool has_active_goal_;
  GoalHandle active_goal_;

  std::vector<std::string> JointNames_;
  int joint_nums_;

public:
  JointTrajectoryExecuter(ros::NodeHandle &n)
    : node_(n)
    , action_server_(node_,
                   "open_manipulator_evk_joint_controller/follow_joint_trajectory",
                   boost::bind(&JointTrajectoryExecuter::goalCB,   this, _1),
                   boost::bind(&JointTrajectoryExecuter::cancelCB, this, _1),
                   false)
    , has_active_goal_(false)
    , last_index_(0)
  {
    using namespace XmlRpc;
    ros::NodeHandle pn("~");

    // get joint names from parameter server
    XmlRpc::XmlRpcValue joint_names;
    if ( !pn.getParam( "joints", joint_names ) )
    {
      ROS_FATAL( "No joints given. (namespace: %s)", pn.getNamespace().c_str() );
      exit(1);
    }
    if ( joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray )
    {
      ROS_FATAL( "Malformed joint specification (namespace: %s)", pn.getNamespace().c_str() );
      exit(1);
    }

    for ( int i = 0; i < joint_names.size(); i++ )
    {
      XmlRpcValue &name_value = joint_names[i];
      if ( name_value.getType() != XmlRpcValue::TypeString )
      {
        ROS_FATAL("Array of joint names should contain all strings. (namespace: %s)", pn.getNamespace().c_str());
        exit(1);
      }
      JointNames_.push_back( (std::string)name_value );
    }

    // store joint nums
    joint_nums_ = JointNames_.size();

    // initialize joint state
    info_.name = JointNames_;
    info_.position.resize( joint_nums_ );
    info_.velocity.resize( joint_nums_ );
    info_.effort.resize( joint_nums_ );
    info_.position[0] = (0.0);
    info_.velocity[0] = (0.0);
    info_.effort[0] = (0.0);

    // initialize pos_internal_ size
    pos_internal_.resize( joint_nums_ );

    // joint state publisher
    pub_joint_state_ =
      node_.advertise<sensor_msgs::JointState>( "joint_states", 1 );

    // arm_up subscriber
    sub_arm_up_ =
      node_.subscribe<std_msgs::Float64MultiArray>( "demo_arm_srv_pos_up",
                                                    1,
                                                    &JointTrajectoryExecuter::subscriberCB,
                                                    this );
    // arm_down publisher
    pub_arm_down_ =
      node_.advertise<std_msgs::Float64MultiArray>( "demo_arm_srv_pos_down", 1 );

    // joint publisher for GAZEBO
    pub_joint_.resize( joint_nums_ );
    for (int i = 0; i < joint_nums_; i++)
    {
      std::string gazebojointname = node_.getNamespace() + "/" + JointNames_[i] + "_position/command";
      pub_joint_[i] =
        node_.advertise<std_msgs::Float64>( gazebojointname, 1 );
    }

    // start action server
    action_server_.start();
  }

  ~JointTrajectoryExecuter()
  {
    // shutdown joint publisher for GAZEBO
    for (int i = 0; i < joint_nums_; i++)
    {
      pub_joint_[i].shutdown();
    }

    // shutdown arm_up/down publisher
    pub_arm_down_.shutdown();
    sub_arm_up_.shutdown();

    // shutdown joint state publisher
    pub_joint_state_.shutdown();
  }

private:
  //
  // @brief 'up' topic subscriber
  //
  void subscriberCB( std_msgs::Float64MultiArray array )
  {
    ros::Time now_time = ros::Time::now();
    struct timespec ts;
    clock_gettime( CLOCK_REALTIME, &ts );
    uint64_t curr_time = (uint64_t)( ts.tv_sec * 1000L + ts.tv_nsec / 1000000L );
    int j;

    // store positions
    for ( j = 0; j < joint_nums_ - NUM_REDUDANT_POSI; j++ )
    {
      pos_internal_[j] = array.data[j + 1];
    }

    // set the value for gripper positions
    pos_internal_[j] = pos_internal_[joint_nums_ - NUM_REDUDANT_POSI];

    // publish 'pos(command)' for GAZEBO
    {
      std_msgs::Float64 msg_;
      for ( int i = 0; i < joint_nums_; i++ )
      {
        msg_.data = pos_internal_[i];
        pub_joint_[i].publish( msg_ );
      }
    }

    // if we don't have a goal, publish current position
    if ( !has_active_goal_ )
    {
      info_.header.stamp = now_time;
      info_.name = JointNames_;
      info_.position.resize( joint_nums_ );
      for ( int i = 0; i < joint_nums_; i++ )
      {
        info_.position[i] = pos_internal_[i];
      }
      pub_joint_state_.publish( info_ );
      return;
    }

    // check period time
    int index = last_index_;
    int max_topics = active_goal_.getGoal()->trajectory.points.size();
    int i;

    uint64_t timediff = curr_time - latest_goal_time_;
    trajectory_msgs::JointTrajectory currTraj = active_goal_.getGoal()->trajectory;
    while ( index < max_topics )
    {
      if ( currTraj.points[index].time_from_start.toSec() * 1000 < timediff )
      { // send feedback
        trajectory_msgs::JointTrajectoryPoint pos;
        trajectory_msgs::JointTrajectoryPoint err;
        pos.positions.resize( joint_nums_ );
        err.positions.resize( joint_nums_ );
        for ( i = 0; i < joint_nums_; i++ )
        {
          pos.positions[i] = pos_internal_[i];
          err.positions[i] = 0.0;
        }

        Feedback fb;
        fb.header = currTraj.header;
        fb.joint_names = JointNames_;
        fb.actual.positions  = pos.positions;
        fb.desired.positions = pos.positions;
        fb.error.positions   = err.positions;
        active_goal_.publishFeedback( fb );

        // send joint states
        info_.header.stamp = now_time;
        info_.name = JointNames_;
        info_.position = pos.positions;
        pub_joint_state_.publish( info_ );

        // publish 'position' 'velocity' 'acceralation' to down topic
        // send positions are 5 points each
        std_msgs::Float64MultiArray _msg;
        _msg.data.push_back( currTraj.points[index].time_from_start.toSec() );
        for ( i = 0; i < joint_nums_ - NUM_REDUDANT_POSI ; i++ )
        {
          if ( i != ( joint_nums_ - NUM_GRIPPER_POSI ) )
          {
            _msg.data.push_back( currTraj.points[index].positions[i] );
          }
          else
          {
            _msg.data.push_back( 0.0 );
          }
        }
        for ( i = 0; i < joint_nums_ - NUM_REDUDANT_POSI ; i++ )
        {
          if ( i != ( joint_nums_ - NUM_GRIPPER_POSI ) )
          {
            _msg.data.push_back( currTraj.points[index].velocities[i] );
          }
          else
          {
            _msg.data.push_back( 0.0 );
          }
        }
        for ( i = 0; i < joint_nums_ - NUM_REDUDANT_POSI ; i++ )
        {
          if ( i != ( joint_nums_ - NUM_GRIPPER_POSI ) )
          {
            _msg.data.push_back( currTraj.points[index].accelerations[i] );
          }
          else
          {
            _msg.data.push_back( 0.0 );
          }
        }
        for ( i = 0; i < joint_nums_ - NUM_REDUDANT_POSI ; i++ )
        {
          if ( i != ( joint_nums_ - NUM_GRIPPER_POSI ) )
          {
            _msg.data.push_back( 0.0 );
          }
          else
          {
            _msg.data.push_back( 0.0 );
          }
        }        
        pub_arm_down_.publish( _msg );
      }
      else
      {
        last_index_ = index;
        return;
      }
      index++;
    }

    // publishing all positions finished
    if ( max_topics <= index )
    {
      has_active_goal_ = false;
      last_index_ = 0;
      active_goal_.setSucceeded();
    }
  }

  void goalCB(GoalHandle gh)
  {
    struct timespec ts;
    clock_gettime( CLOCK_REALTIME, &ts );
    uint64_t curr_time = (uint64_t)( ts.tv_sec * 1000L + ts.tv_nsec / 1000000L );
    latest_goal_time_ = curr_time;
    gh.setAccepted();
    active_goal_ = gh;
    has_active_goal_ = true;
  }

  void cancelCB(GoalHandle gh)
  {
    if (active_goal_ == gh)
    {
      // mark the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }
  }
};

/**
 * @brief main
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "custom_joint_controller");
  ros::NodeHandle node;
  JointTrajectoryExecuter jte(node);

  ros::spin();

  return 0;
}
