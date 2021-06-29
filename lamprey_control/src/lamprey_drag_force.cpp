#include <math.h>
#include <iostream>
#include <boost/math/special_functions/sign.hpp>

#include "lamprey_drag_force.h"

void LampreyDragForce::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Store the pointer to the model
    this->model = _model;

    // Save pointers
    this->world = this->model->GetWorld();

    // Load pid parameters
    this->config = YAML::LoadFile(std::string(std::getenv("HBP")) + "/GazeboRosPackages/src/lamprey_control/src/control.yaml");

    // Read drag coefficients
    this->dragCoefficients = ignition::math::Vector3d(this->config["dragCoefficients"]["xCoeff"].as<double>(),
                                                      this->config["dragCoefficients"]["yCoeff"].as<double>(),
                                                      this->config["dragCoefficients"]["zCoeff"].as<double>());

    // subscribe to the gazebo physics update event
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&LampreyDragForce::OnPhysicsUpdate, this));

    gzdbg << "Plugin model name: " << this->model->GetName() << "\n";
}

ignition::math::Vector3d sign(const ignition::math::Vector3d &input)
{
    auto x = boost::math::sign<double>(input.X());
    auto y = boost::math::sign<double>(input.Y());
    auto z = boost::math::sign<double>(input.Z());
    return ignition::math::Vector3d(x, y, z);
}

void LampreyDragForce::OnPhysicsUpdate()
{
    boost::mutex::scoped_lock lock(this->mutex);
    common::Time currTime = this->model->GetWorld()->SimTime();
    common::Time stepTime = currTime - this->prevUpdateTime;
    this->prevUpdateTime = currTime;

    // apply drag forces
    for (int i = 0; i < this->model->GetLinks().size(); i++)
    {
        auto link = this->model->GetLinks()[i];

        // Rotation of the link frame in Cartesian coords
        auto urdfOrientation = ignition::math::Matrix3d(link->WorldPose().Rot());
        auto linkUrdfOrientationInverse = urdfOrientation.Transposed();

        // Velocity of CoM in Cartesian coords
        auto linkWorldCoGVelocity = link->WorldLinearVel();

        auto speedBarrierVelocity = ignition::math::Vector3d(0.0, 0.0, 0.0);
        // Uncomment if you want to change the water speed
        /*
        if (condition)
        {
            speedBarrierVelocity = ignition::math::Vector3d(0.2, 0.0, 0.0);
        } */

        // Velocity of CoM in link frame coords
        auto relativeVelocityCoM = linkUrdfOrientationInverse * (link->WorldCoGLinearVel() + speedBarrierVelocity);
        auto dragForce = sign(relativeVelocityCoM) * relativeVelocityCoM * relativeVelocityCoM * this->dragCoefficients;

        // Center of gravity offset relative to link frame
        auto CoGOffsetInLocalFrame = link->GetInertial()->Pose().Pos();
        if (link->GetScopedName().find("body_module") != std::string::npos)
        {
            link->AddLinkForce(dragForce, CoGOffsetInLocalFrame);
        }
    }
}

GZ_REGISTER_MODEL_PLUGIN(LampreyDragForce);
