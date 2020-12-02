#ifndef _LAMPREY_DRAG_FORCE_HH_
#define _LAMPREY_DRAG_FORCE_HH_

// gazebo related
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// C++ stl
#include <memory>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// external libs
#include <yaml-cpp/yaml.h>

using namespace gazebo;

class LampreyDragForce : public ModelPlugin
{
public:
    LampreyDragForce(){};
    ~LampreyDragForce()
    {
    };

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

protected:
    virtual void OnPhysicsUpdate();

private:
    // Gazebo related
    physics::ModelPtr model;
    physics::WorldPtr world;
    event::ConnectionPtr updateConnection;

    YAML::Node config;
    ignition::math::Vector3d dragCoefficients;
    common::Time prevUpdateTime;
    boost::mutex mutex;
};

#endif