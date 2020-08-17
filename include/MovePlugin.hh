#ifndef _GAZEBO_MOVE_PLUGIN_HH_
#define _GAZEBO_MOVE_PLUGIN_HH_

#include <functional>
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <geometry_msgs/Vector3.h>
#include <gazebo/math/Pose.hh>


namespace gazebo
{
  class MovePlugin : public ModelPlugin
  {

    /// \brief Constructor.
    public: MovePlugin();

    /// \brief Destructor.
    public: virtual ~MovePlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the Move sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Callback that receives the Move sensor's update signal.
    private: virtual void info_callback(const geometry_msgs::Vector3& position);


    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // Node Handler
    private:ros::NodeHandle* node_handle_;

    // Subscriber Topic Name
    private:std::string subscriber_topic_name ;

    // Subscriber
    private:ros::Subscriber _subscriber;
  };
}
#endif