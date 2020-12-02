README
======

To setup the experiment with the spiking CPG you need to have the Neurorobotics Platform (NRP) installed or your machine. Instructions on how to do that here [NRP website](http://www.neurorobotics.net)!

## To simulate the isolated spinal cord model ##
    cd Models/brain_model
    source ~/.opt/platform_venv/bin/activate
    nengo

This will open a nengo GUI window where you can select the model called Ijspeert_CPG and click play to simulate it. There are two options, pure math, or with spiking neurons. The default version is with spiking neurons. To activate the math version uncomment these lines in the script.
    
    for ens in model.ensembles:
        ens.neuron_type=nengo.Direct()
        
Alternatively if you want to run the script with the command line uncomment these lines:

    with nengo.Simulator(model) as sim:
        start_time = time.clock()
        sim.run(10)
        print('Time Lasted : {0}'.format(time.clock()-start_time))
        
Optional: If you want to plot in the command line version uncomment these lines

    probe_osc = nengo.Probe(output,synapse=0.3)
    AND
    plt.plot(sim.trange(), sim.data[probe_osc])
    plt.show()

To run the script with the command line just do 

    python ijspeert_CPG.py

Running the model without the neuromechanical simulation does not depend on the NRP installation, so if you just install nengo in your machine running the script should work. To install nengo (you should have pip installed, and it's better to install it in a virtual environment, how to setup a virtual environment here https://packaging.python.org/guides/installing-using-pip-and-virtual-environments/).
    
    # optional if in a venv
    source location_of_your_venv

    pip install nengo

## Running the model on SpiNNaker #
If you have installed the NRP then SpiNNaker is already configured in your system. Else you will find instructions on setting up a SpiNNaker board here http://apt.cs.manchester.ac.uk/projects/SpiNNaker/

Install nengo_spinnaker
    
    # optional if in a venv
    source location_of_your_venv
    pip install nengo_spinnaker

add the line on top of the script
    import nengo_spinnaker    

You can only run the SpiNNaker version with the command line. In the script uncomment the lines
    
    with nengo.Simulator(model) as sim:
        start_time = time.clock()
        sim.run(10)
        print('Time Lasted : {0}'.format(time.clock()-start_time))

and change the lines
    with nengo.Simulator(model) as sim:
    to 
    with nengo_spinnaker.Simulator(model) as sim:

Then run the script as usual
    cd Models/brain_model
    source ~/.opt/platform_venv/bin/activate
    python ijspeert_CPG.py

## Running the model on Intel Loihi #
Setup the Loihi board with instructions found in the INRC wiki

Install nengo_loihi
    
    # optional if in a venv
    source location_of_your_venv
    pip install nengo_loihi
    
Similar to SpiNNaker, you can only run the Loihi version with the command line. In the script uncomment the lines
    
    with nengo.Simulator(model) as sim:
        start_time = time.clock()
        sim.run(10)
        print('Time Lasted : {0}'.format(time.clock()-start_time))

add the line on top of the script
    import nengo_loihi

and change the lines
    with nengo.Simulator(model) as sim:
    to 
    with nengo_loihi.Simulator(model) as sim:

Then run the nengo version of the script
    cd Models/brain_model
    source ~/.opt/platform_venv/bin/activate
    python ijspeert_CPG_loihi.py

There is some commented code in the script if you want to perform some energy benchmarks

## Setting up the NRP experiment ##
First you need to copy the experiment and model data into the correct folders

    # Copy over models
    cp Models/brain_model/ijspeert_CPG.py $HBP/Models/brain_model
    cd $HBP/Models && ./create-symlinks.sh
    
    # Copy over experiment
    cp Experiments/amphibot_swimming_spiking_CPG $HBP/Experiments
    
    # Install the robot controller
    cp lamprey_control $HBP/GazeboRosPackages/src
    cd $HBP/GazeboRosPackages
    catkin_make
    
Then run the NRP:
    
    cle-nginx
    cle-start
    
Open a browser window, go to https://localhost:9000/#/esv-private. From the list of template experiments you should see the amphibot swimming spiking CPG experiment. Press clone. You should get redirected to the page of cloned experiments. Find the amphibot experiment and press launch. If everything goes well you should land in the 3D Scene and be able to play around. Click play to start the simulation - should take some time as the neural net is being initialized - the robot should start swimming, depending on your CPU it might be ~5 to 10 times slower than real-time.