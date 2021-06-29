#include <math.h>
#include <iostream>
#include <boost/math/special_functions/sign.hpp>
#include <std_msgs/Float64.h>
#include "lamprey_simple_fluids_controller.h"
#include <iomanip>
std::ofstream file;

inline double round(double val)
{
    if (val < 0)
        return ceil(val - 0.5);
    return floor(val + 0.5);
}

void LampreyController::ReadCSVData(const std::string &filePath)
{

    csvFile.open(filePath);
    if (!csvFile.is_open())
        throw std::runtime_error("Could not open file");

    std::vector<std::vector<double>> data;
    int i = 0;
    while (csvFile)
    {
        std::string s;
        if (!std::getline(csvFile, s))
            break;

        std::istringstream ss(s);
        std::vector<double> record;

        while (ss)
        {
            std::string s;
            if (!getline(ss, s, ','))
                break;
            record.push_back(std::stod(s));
        }
        int index = i;
        record.pop_back();
        std::pair<int, std::vector<double>> result(i, record);
        csvPositions.insert(result);
        i++;
    }
}

void LampreyController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Store the pointer to the model
    this->model = _model;

    // Save pointers
    this->world = this->model->GetWorld();

    // Initialize the ROS node for the gazebo client if necessary
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "lampreyController",
                  ros::init_options::NoSigintHandler);
    }
    rosnode.reset(new ros::NodeHandle());

    // setup a ros topic for the velocity and position targets
    std::string lampreyTargetsTopicName = "/lamprey_targets";
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<gazebo_msgs::JointsPositions>(lampreyTargetsTopicName, 1,
                                                                   boost::bind(&LampreyController::OnTargets, this, _1),
                                                                   ros::VoidPtr(), &queue);
    targetsSub = rosnode->subscribe(so);
    model->SaveControllerActuatorRosTopics(lampreyTargetsTopicName, "gazebo_msgs::JointPositions");

    this->callbackQueueThread =
        std::thread(std::bind(&LampreyController::QueueThread, this));

    std::string pubTopicName = this->model->GetName() + "/rotation_error";
    rotationErrorPub = rosnode->advertise<std_msgs::Float64>(pubTopicName, 1);

    // Load configuration file
    this->config = YAML::LoadFile(std::string(std::getenv("HBP")) + "/GazeboRosPackages/src/lamprey_control/src/control.yaml");

    // Set up PID parameters
    int j = 0;
    for (int i = 0; i < this->model->GetJoints().size(); ++i)
    {
        std::shared_ptr<Joint> jointPtr = std::make_shared<Joint>();
        jointPtr->joint = this->model->GetJoints()[i];
        if (!jointPtr->joint->HasType(gazebo::physics::Joint::FIXED_JOINT))
        {
            std::string jointId = "joint_c2m_" + std::to_string(j);
            auto posP = this->config["joints"][jointId]["pid"]["position"]["p"].as<double>();
            auto posI = this->config["joints"][jointId]["pid"]["position"]["i"].as<double>();
            auto posD = this->config["joints"][jointId]["pid"]["position"]["d"].as<double>();

            auto velP = this->config["joints"][jointId]["pid"]["velocity"]["p"].as<double>();
            auto velI = this->config["joints"][jointId]["pid"]["velocity"]["i"].as<double>();
            auto velD = this->config["joints"][jointId]["pid"]["velocity"]["d"].as<double>();

            this->model->GetJointController()->SetPositionPID(jointPtr->joint->GetScopedName(), common::PID(posP, posI, posD));
            this->model->GetJointController()->SetVelocityPID(jointPtr->joint->GetScopedName(), common::PID(velP, velI, velD));
            joints.push_back(jointPtr);
            j++;
        }
    }

    // Ideal CPG Parameters
    this->amplitude = this->config["oscillation"]["amplitude"].as<double>();
    this->frequency = this->config["oscillation"]["frequency"].as<double>();
    this->bias = this->config["oscillation"]["bias"].as<double>();
    this->phase = this->config["oscillation"]["phase"].as<double>();
    this->omega = 2 * M_PI * frequency;
    gzdbg << "Plugin model name: " << this->model->GetName() << "\n";

    //Uncomment if you want to read recorded data from a file
    //this->ReadCSVData(std::string(std::getenv("HBP")) + "/GazeboRosPackages/src/lamprey_control/results/results.csv");
}

void LampreyController::QueueThread()
{
    static const double timeout = 0.01;
    while (rosnode->ok())
    {
        queue.callAvailable(ros::WallDuration(timeout));
    }
}

void LampreyController::FirstLinkRotationErrorPub(const gazebo_msgs::JointsPositions::ConstPtr &msg)
{
    double currentError = 0;
    if (!firstLinkRotationMemory.empty() && iterations > 501)
        firstLinkRotationMemory.pop_front();

    firstLinkRotationMemory.push_back(this->model->GetLinks()[0]->WorldCoGPose().Rot().Z());
    currentError = msg->positions[msg->positions.size() - 1] - averageFirstLinkRotation;

    double sum = std::accumulate(firstLinkRotationMemory.begin(), firstLinkRotationMemory.end(), 0.0);
    averageFirstLinkRotation = sum / firstLinkRotationMemory.size();

    std_msgs::Float64 errorMessage;

    errorMessage.data = currentError;
    this->rotationErrorPub.publish(errorMessage);
}

void LampreyController::OnTargets(const gazebo_msgs::JointsPositions::ConstPtr &msg)
{
    boost::mutex::scoped_lock lock(this->mutex);
    common::Time currTime = this->model->GetWorld()->SimTime();
    common::Time stepTime = currTime - this->prevUpdateTime;
    this->prevUpdateTime = currTime;

    if (stepTime.Double() > 0.0)
    {
        // for every joint read (or compute) the target position,  and
        // apply as a target with a control method
        for (int i = 0; i < this->joints.size(); i++)
        {
            auto joint = this->joints[i]->joint;
            double posCmd = 0.0, velCmd = 0.0, currFilteredValue = 0.0;
            auto currPosition = joint->Position(0);
            auto currVelocity = joint->GetVelocity(0);

            // read the target position from the ROS topic
            posCmd = msg->positions[i];
            // ideal signal
            //posCmd = amplitude * sin(omega * currTime.Double() - phase * i) + bias;
            //velCmd = omega * amplitude * cos(omega * currTime.Double() - phase * i);

            // Finite differences scheme of first order for the velocity
            // velCmd = (posCmd - positionsMemory[i]) / stepTime.Double();

            // 4 Control Methods:
            // 1: Gazebo Default PID controller
            // 2: Explicitly setting the joint to a position (works best when setting the velocity as well)
            // 3: Explicitly setting the velocity of the joint (works best when setting the position as well)
            // 4: PD control based on the PID controller equation provided by pybullet
            if (this->config["method"]["GazeboPIDControl"].as<bool>())
            {
                double target = posCmd;
                this->model->GetJointController()->SetPositionTarget(joint->GetScopedName(), target);
            }

            if (this->config["method"]["directPositionControl"].as<bool>())
                joint->SetPosition(0, posCmd, true);

            if (this->config["method"]["velocityControl"].as<bool>())
            {
                joint->SetParam("fmax", 0, 100.0);
                joint->SetParam("vel", 0, velCmd);
            }

            if (this->config["method"]["PDControl"].as<bool>())
            {
                auto forceTarget = this->joints[i]->pidPosition.GetPGain() * ((posCmd - currPosition) / stepTime.Double()) + this->joints[i]->pidPosition.GetDGain() * (velCmd - currVelocity);
                joint->SetForce(0, forceTarget);
            }
        }

        iterations++;
        this->FirstLinkRotationErrorPub(msg);
    }
}

GZ_REGISTER_MODEL_PLUGIN(LampreyController);
