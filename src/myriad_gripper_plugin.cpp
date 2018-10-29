//*****************************************************************************
//
// File Name	: 'myriad_gripper_plugin.cpp'
// Author	: Steve NGUYEN
// Contact      : steve.nguyen.000@gmail.com
// Created	: dimanche, septembre 16 2018
// Revised	:
// Version	:
// Target MCU	:
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//
// Notes:	based on the vaccum gripper plugin
//
//*****************************************************************************


#include "myriad_gripper_plugin.h"
#include <algorithm>
#include <assert.h>

#include <std_msgs/Bool.h>



namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosMyriadGripper);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosMyriadGripper::GazeboRosMyriadGripper()
{
  connect_count_ = 0;
  // status_ = false;
  status_ = true; //on by default
  
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosMyriadGripper::~GazeboRosMyriadGripper()
{
  update_connection_.reset();

  // Custom Callback Queue
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();

  delete rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosMyriadGripper::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROS_INFO_NAMED("myriad_gripper", "Loading gazebo_ros_myriad_gripper");

  // Set attached model;
  parent_ = _model;

  // Get the world name.
  world_ = _model->GetWorld();

  // load parameters
  robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL_NAMED("myriad_gripper", "myriad_gripper plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

  link_ = _model->GetLink(link_name_);
  if (!link_)
  {
    std::string found;
    physics::Link_V links = _model->GetLinks();
    for (size_t i = 0; i < links.size(); i++) {
      found += std::string(" ") + links[i]->GetName();
    }
    ROS_FATAL_NAMED("myriad_gripper", "gazebo_ros_myriad_gripper plugin error: link named: %s does not exist", link_name_.c_str());
    ROS_FATAL_NAMED("myriad_gripper", "gazebo_ros_myriad_gripper plugin error: You should check it exists and is not connected with fixed joint");
    ROS_FATAL_NAMED("myriad_gripper", "gazebo_ros_myriad_gripper plugin error: Found links are: %s", found.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL_NAMED("myriad_gripper", "myriad_gripper plugin missing <serviceName>, cannot proceed");
    return;
  }
  else
    topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("myriad_gripper", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                           << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  rosnode_ = new ros::NodeHandle(robot_namespace_);

  // Custom Callback Queue
  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<std_msgs::Bool>(
    topic_name_, 1,
    boost::bind(&GazeboRosMyriadGripper::Connect, this),
    boost::bind(&GazeboRosMyriadGripper::Disconnect, this),
    ros::VoidPtr(), &queue_);
  pub_ = rosnode_->advertise(ao);

  // Custom Callback Queue
  ros::AdvertiseServiceOptions aso1 =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      "on", boost::bind(&GazeboRosMyriadGripper::OnServiceCallback,
                        this, _1, _2), ros::VoidPtr(), &queue_);
  srv1_ = rosnode_->advertiseService(aso1);
  ros::AdvertiseServiceOptions aso2 =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      "off", boost::bind(&GazeboRosMyriadGripper::OffServiceCallback,
                         this, _1, _2), ros::VoidPtr(), &queue_);
  srv2_ = rosnode_->advertiseService(aso2);

  // Custom Callback Queue
  callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosMyriadGripper::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&GazeboRosMyriadGripper::UpdateChild, this));

  ROS_INFO_NAMED("myriad_gripper", "Loaded gazebo_ros_myriad_gripper");
}

bool GazeboRosMyriadGripper::OnServiceCallback(std_srvs::Empty::Request &req,
                                               std_srvs::Empty::Response &res)
{
  if (status_) {
    ROS_WARN_NAMED("myriad_gripper", "gazebo_ros_myriad_gripper: already status is 'on'");
  } else {
    status_ = true;
    ROS_INFO_NAMED("myriad_gripper", "gazebo_ros_myriad_gripper: status: off -> on");
  }
  return true;
}
bool GazeboRosMyriadGripper::OffServiceCallback(std_srvs::Empty::Request &req,
                                                std_srvs::Empty::Response &res)
{
  if (status_) {
    status_ = false;
    ROS_INFO_NAMED("myriad_gripper", "gazebo_ros_myriad_gripper: status: on -> off");
  } else {
    ROS_WARN_NAMED("myriad_gripper", "gazebo_ros_myriad_gripper: already status is 'off'");
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosMyriadGripper::UpdateChild()
{
  std_msgs::Bool grasping_msg;
  grasping_msg.data = false;
  if (!status_) {
    pub_.publish(grasping_msg);
    return;
  }
  // apply force
  lock_.lock();
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Pose3d parent_pose = link_->WorldPose();
  
  physics::Model_V models = world_->Models();
#else
  ignition::math::Pose3d parent_pose = link_->GetWorldPose().Ign();
  physics::Model_V models = world_->GetModels();
#endif
  for (size_t i = 0; i < models.size(); i++) {

    // if (models[i]->GetName() == link_->GetName() ||
    //     models[i]->GetName() == parent_->GetName())
    // {
    // continue;
    // }

    // if (models[i]->GetName() == link_->GetModel()->GetName() )
    // {
    //   continue;
    // }
    
    physics::Link_V links = models[i]->GetLinks();
    for (size_t j = 0; j < links.size(); j++) {
      // ROS_INFO_NAMED("myriad_gripper", "%d",links[j]->GetName().c_str());

      
      if (links[j]->GetModel()->GetName() == link_->GetModel()->GetName() || !(links[j]->GetName().find("effector") != std::string::npos) )
        // if (links[j]->GetModel()->GetName() == link_->GetModel()->GetName()  )
      {
        continue;
      }


#if GAZEBO_MAJOR_VERSION >= 8
      ignition::math::Pose3d link_pose = links[j]->WorldPose();
#else
      ignition::math::Pose3d link_pose = links[j]->GetWorldPose().Ign();
#endif
      ignition::math::Pose3d diff = parent_pose - link_pose;
      double norm = diff.Pos().Length();
      // std::cerr<<"GRIP: "<< links[j]->GetModel()->GetName()<<" "<<links[j]->GetName()<<" "<<link_->GetModel()->GetName()<<" "<<link_->GetName()<<" "<<norm<<" "<<parent_pose.Pos()<<" "<<link_pose.Pos()<<std::endl;      
      if (norm < 0.05) {
        double norm_force = 1 / norm;
        
#if GAZEBO_MAJOR_VERSION >= 8
        links[j]->SetLinearVel(link_->WorldLinearVel());
        links[j]->SetAngularVel(link_->WorldAngularVel());
#else
        links[j]->SetLinearVel(link_->GetWorldLinearVel());
        links[j]->SetAngularVel(link_->GetWorldAngularVel());
#endif

// double norm_force = 1000;

        
        if (norm < 0.05) {
// apply friction like force
// TODO(unknown): should apply friction actually
          link_pose.Set(parent_pose.Pos(), link_pose.Rot());
          links[j]->SetWorldPose(link_pose);
        }
        
        if (norm < 0.025) {
          // norm_force=norm;
          norm_force=0.0; 
          // norm_force*=norm*0.001;
          
            }
        
                
        
        if (norm_force > 1) {
          norm_force = 1;  // max_force
        }
        ignition::math::Vector3d force = norm_force * diff.Pos().Normalize();
        // ignition::math::Vector3d force;
        // force[0]=1000.0;
        // force[1]=1000.0;
        // force[2]=0;
        
        // std::cerr<<"GRIP: "<<link_->GetWorld()->Name()<<" "<< link_->GetName()<<" "<<links[j]->GetName()<<" "<<link_->WorldLinearVel()<<" "<<norm<<" "<<force<<std::endl;

        links[j]->AddForce(force);
        grasping_msg.data = true;
      }
    }
  }
  pub_.publish(grasping_msg);
  lock_.unlock();
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosMyriadGripper::QueueThread()
{
  static const double timeout = 0.01;

  while (rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosMyriadGripper::Connect()
{
  this->connect_count_++;
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosMyriadGripper::Disconnect()
{
  this->connect_count_--;
}

} // namespace gazebo
