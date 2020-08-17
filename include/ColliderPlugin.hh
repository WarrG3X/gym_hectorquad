#ifndef _GAZEBO_COLLIDER_PLUGIN_HH_
#define _GAZEBO_COLLIDER_PLUGIN_HH_

#include <string>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>


namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class ColliderPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: ColliderPlugin();

    /// \brief Destructor.
    public: virtual ~ColliderPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the contact sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;

    // Node Handler
    private:ros::NodeHandle* node_handle_;

    // Count Publisher
    private:ros::Publisher _count_publisher;

    // String Publisher
    private:ros::Publisher _string_publisher;
  };
}
#endif