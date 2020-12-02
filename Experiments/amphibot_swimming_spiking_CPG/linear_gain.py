import numpy as np
"""
This module contains the transfer function which is responsible for applying the linear gain
to the output of the nengo brain
"""
@nrp.MapSpikeSink("theta0", nrp.brain.theta_dict[0], nrp.raw_signal)
@nrp.MapSpikeSink("theta1", nrp.brain.theta_dict[1], nrp.raw_signal)
@nrp.MapSpikeSink("theta2", nrp.brain.theta_dict[2], nrp.raw_signal)
@nrp.MapSpikeSink("theta3", nrp.brain.theta_dict[3], nrp.raw_signal)
@nrp.MapSpikeSink("theta4", nrp.brain.theta_dict[4], nrp.raw_signal)
@nrp.MapSpikeSink("theta5", nrp.brain.theta_dict[5], nrp.raw_signal)
@nrp.MapSpikeSink("theta6", nrp.brain.theta_dict[6], nrp.raw_signal)
@nrp.MapSpikeSink("theta7", nrp.brain.theta_dict[7], nrp.raw_signal)
@nrp.MapSpikeSink("theta8", nrp.brain.theta_dict[8], nrp.raw_signal)
@nrp.MapSpikeSink("theta9", nrp.brain.theta_dict[9], nrp.raw_signal)
@nrp.MapSpikeSink("theta10", nrp.brain.theta_dict[10], nrp.raw_signal)
@nrp.MapSpikeSink("theta11", nrp.brain.theta_dict[11], nrp.raw_signal)
@nrp.MapSpikeSink("theta12", nrp.brain.theta_dict[12], nrp.raw_signal)
@nrp.MapSpikeSink("theta13", nrp.brain.theta_dict[13], nrp.raw_signal)
@nrp.MapSpikeSink("theta14", nrp.brain.theta_dict[14], nrp.raw_signal)
@nrp.MapSpikeSink("theta15", nrp.brain.theta_dict[15], nrp.raw_signal)
@nrp.MapRobotPublisher('lamprey_positions',Topic('/lamprey_targets', gazebo_msgs.msg.JointPositions))
@nrp.Neuron2Robot()
def linear_gain(t,lamprey_positions, theta0, theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta8,theta9, theta10, theta11, theta12, theta13, theta14, theta15):
    """
    The transfer function which calculates the target position for lamprey joints.

    :param t: the current simulation time
    :param out: connection to the 16 dimensional output signal of the Nengo brain
    :return: a gazebo_msgs/JointPositions message setting the target position for the lamprey joints.
    """
    thetas = [theta0.value, theta1.value, theta2.value, theta3, theta4, theta5, theta6, theta7, theta8, theta9, theta10, theta11, theta12, theta13, theta14, theta15]
    msg = gazebo_msgs.msg.JointPositions()
    msg.names = ''
    targetRotation = 0.0
    ampl = 0.15
    jointsPositions = [
        ampl * (theta0.value[0]-theta1.value[0]),ampl * (theta2.value[0]-theta3.value[0]),ampl * (theta4.value[0]-theta5.value[0]),ampl * (theta6.value[0]-theta7.value[0]),ampl * (theta8.value[0]-theta9.value[0]),ampl * (theta10.value[0]-theta11.value[0]),ampl * (theta12.value[0]-theta13.value[0]),ampl * (theta14.value[0]-theta15.value[0])]
    jointsPositions = np.append(jointsPositions,[targetRotation])
    msg.positions = jointsPositions
    lamprey_positions.send_message(msg)