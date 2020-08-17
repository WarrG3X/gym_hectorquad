#include "MovePlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(MovePlugin)

/////////////////////////////////////////////////
MovePlugin::MovePlugin() : ModelPlugin()
{
}

/////////////////////////////////////////////////
MovePlugin::~MovePlugin()
{
}

/////////////////////////////////////////////////
void MovePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

  // Check ROS
  if (!ros::isInitialized())
  {
    ROS_INFO("ROS should be initialized first!");
    return;
  }

  // Init Node
  this->node_handle_ = new ros::NodeHandle("move_plugin_node");

  // Get Topic Name
  this->subscriber_topic_name = _sdf->GetElement("topic-name")->GetValue()->GetAsString() ;

  //Subscriber
  _subscriber = this->node_handle_->subscribe(subscriber_topic_name.c_str(),1,&MovePlugin::info_callback,this);

  // Store the pointer to the model
  this->model = _model;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&MovePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
// Called by the world update start event
void MovePlugin::OnUpdate()
{
  // Apply a small linear velocity to the model.
  // this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
  // this->model->SetWorldPose(gazebo::math::Pose(1,1,1,0,0,0));
}

/////////////////////////////////////////////////
// Callback function for Subscriber
void MovePlugin::info_callback(const geometry_msgs::Vector3& position){
  
  gazebo::math::Pose loc(position.x,position.y,position.z,0,0,0);
  this->model->SetWorldPose(loc);
  ROS_WARN("Got Pose");
}