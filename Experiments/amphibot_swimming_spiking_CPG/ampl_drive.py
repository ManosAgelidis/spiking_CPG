@nrp.MapSpikeSource("drive", nrp.brain.drive, nrp.raw_signal)
@nrp.MapRobotSubscriber('rotation_error',Topic('/apmhibot/rotation_error', std_msgs.msg.Float64))
@nrp.Robot2Neuron()
def ampl_control (t,drive,amplitude_error):
    kP = 20.0
    correction = 0.0
    drive.value[0] = 3
    drive.value[1] = 3
    if amplitude_error.value is not None:
        correction = kP * amplitude_error.value.data
    # right
    if correction > 0.0:
        drive.value[0] = 3 + correction
    # left
    elif correction < 0.0:
        drive.value[1] = 3 + correction