#include "ColliderPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ColliderPlugin)

/////////////////////////////////////////////////
ColliderPlugin::ColliderPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ColliderPlugin::~ColliderPlugin()
{
}

/////////////////////////////////////////////////
void ColliderPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Init Node
  this->node_handle_ = new ros::NodeHandle("collider_plugin_node");

  // Count Publisher
  this->_count_publisher= this->node_handle_->advertise<std_msgs::Int16>("/collision_count", 1);

  // String Publisher
  this->_string_publisher= this->node_handle_->advertise<std_msgs::String>("/collision_status", 1);

  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ColliderPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ColliderPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

}

/////////////////////////////////////////////////
void ColliderPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  std_msgs::Empty message;
  std_msgs::Int16 count;
  std_msgs::String string;

  string.data = "YES!";
  count.data = contacts.contact_size();

  this->_count_publisher.publish(count); 

  if(contacts.contact_size()>0){
    this->_string_publisher.publish(string); 

  }
  
}