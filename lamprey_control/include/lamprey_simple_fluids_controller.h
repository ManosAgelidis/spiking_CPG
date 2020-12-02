#ifndef _LAMPREY_SIMPLE_FLUIDS_CONTROLLER_HH_
#define _LAMPREY_SIMPLE_FLUIDS_CONTROLLER_HH_

// gazebo related
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_msgs/JointPositions.h>

// C++ stl
#include <memory>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// ros related
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/callback_queue.h>

// external libs
#include <yaml-cpp/yaml.h>
#include <gram_savitzky_golay/gram_savitzky_golay.h>

#include "Plotter.h"

using namespace gazebo;

class Joint
{
public:
    Joint() : pidPosition(
                  1e1,    // _P
                  1e0,    // _I
                  0,      // _D
                  0.1,    // _imax
                  -0.1,   // _imin
                  10.0,   // _cmdMax
                  -10.0), // _cmdMin
              pidVelocity(
                  1e-2, // _P
                  1e-3, // _I
                  0,    // _D
                  0.1,  // _imax
                  -0.1, // _imin
                  10.0, // _cmdMax
                  -10.0)
    {
    }

    void update()
    {
        // this->name = this->joint->GetName();
#if GAZEBO_MAJOR_VERSION >= 8
        this->position = this->joint->Position(0);
#else
        this->position = this->joint->GetAngle(0).Radian();
#endif
        this->velocity = this->joint->GetVelocity(0);
        return;
    }

    void set_force(double force)
    {
        if (!this->joint->HasType(gazebo::physics::Joint::FIXED_JOINT))
            this->joint->SetForce(0, force);
        return;
    }

    double control(double positionCmd, double velocityCmd, bool verbose = false)
    {
        this->update();
        this->pidPosition.SetCmd(0);
        double errPos = this->position - positionCmd;
        double cmdPos = this->pidPosition.Update(
            errPos,
            0.001);
        double errVel = this->velocity - velocityCmd;
        double cmdVel = this->pidVelocity.Update(
            errVel,
            0.001);
        this->set_force(cmdPos + cmdVel);
        return cmdPos + cmdVel;
    }

    gazebo::common::PID pidPosition;
    gazebo::common::PID pidVelocity;
    gazebo::physics::JointPtr joint;

private:
    double position, velocity;
};

class LampreyController : public ModelPlugin
{
public:
    LampreyController(){};
    ~LampreyController(){};

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void OnTargets(const gazebo_msgs::JointPositions::ConstPtr &msg);
    void QueueThread();

private:
    // Gazebo related
    physics::ModelPtr model;
    physics::WorldPtr world;
    event::ConnectionPtr updateConnection;
    boost::mutex mutex;

    // ros related
    std::shared_ptr<ros::NodeHandle> rosnode;
    ros::Subscriber targetsSub;
    ros::Publisher rotationErrorPub;
    ros::CallbackQueue queue;
    std::thread callbackQueueThread;

    YAML::Node config;
    double amplitude, frequency, omega, bias, phase;
    std::vector<std::shared_ptr<Joint>> joints;
    gazebo::common::Time prevUpdateTime;

    // iterations counter
    int iterations;
    void FirstLinkRotationErrorPub(const gazebo_msgs::JointPositions::ConstPtr &msg);
    std::deque<double> firstLinkRotationMemory;
    double averageFirstLinkRotation = 0.0;
    
    // External data read
    std::ifstream csvFile;
    void ReadCSVData(const std::string &filePath);
    std::map<int,std::vector<double>> csvPositions;
};

#endif