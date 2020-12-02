# We use the same differential equation as before:
#    dx0/dt = (mu - r**2)*x0 - w*x1 + epsilon*F
#    dx1/dt = (mu . r**2)*x1 - w*x0
# where r is the radius of the circle and s is the speed (in radians per
# second).
# But, in this case, we make the Ensemble be 3-dimensional and use the third
# dimension (x[2]) to represent s.  You can control it with a separate input.
# This shows how neurons can affect the pattern of activity of another
# group of neurons.
#\[Mu] = 1;
#s1 = NDSolve[{x'[t] == (\[Mu] - (x[t]^2 + y[t]^2))*x[t] + \[Omega][t]*y[t] +  k*Sin[ForcingFreq*t],
#              y'[t] == (\[Mu] - (x[t]^2 + y[t]^2))*y[t] - \[Omega][t]*x[t],
#              \[Omega]'[t] == k*Sin[ForcingFreq*t]*y[t]/(Sqrt[x[t]^2 + y[t]^2]),
#              x[0] == x0,
#              y[0] == v0,
#              \[Omega][0] == \[Omega]0}, #differential equations
#     {x, y, \[Omega]},                   #variables
#     {t, 0, 200*Pi}];                   #range?
import nengo
import numpy as np
import time
import matplotlib.pyplot as plt

def get_neighbor_pairs(np_array):
#Takes in a 2D numpy array and returns a set of sets containing a value pair of every neighbor in the array. Each value pair will appear only once:
    neighbor_pairs = set()
    for (x, y), item in np.ndenumerate(np_array):
        # Check left.
        if x > 0:
            var = np_array[x-1, y]
            neighbor_pairs.add(frozenset({item,var}))
        # Check right.
        try:
            var = np_array[x+1, y]
            neighbor_pairs.add(frozenset({item,var}))
        except:
            pass
        # Check top.
        try:
            var = np_array[x, y+1]
            neighbor_pairs.add(frozenset({item,var}))
        except:
            pass
        # Check bottom.
        if y > 0:
            var = np_array[x, y-1]
            neighbor_pairs.add(frozenset({item,var}))
    return neighbor_pairs

circuit = nengo.Network()
with circuit:
    def high_level_drive_R(d):
        d_scaled = d#d * 10
        c_R1 = 0.065    
        c_R0 = 0.196
        R_sat = 0
        d_low = 1.0
        d_high = 5.0
        if d_scaled < d_low or d_scaled > d_high:
            return R_sat
        else:
            return c_R1*d_scaled+c_R0
    def high_level_drive_v(d):
        d_scaled  = d#10 * d
        c_v1 = 0.2
        c_v0 = 0.8
        v_sat = 0
        d_low = 1.0
        d_high = 5.0
        if d_scaled < d_low or d_scaled > d_high:
            return v_sat
        else:
            return c_v1*d_scaled+c_v0
    
    def oscillator(x):
        if x[0]==0:
            x[0]+=0.01
        gamma = 5 # defines how fast it will leave the stable point at [0,0]
        mu = 2*x[3] # amplitude of the oscillations, the lower this value is the slower the two lateral oscillators diverge
        r = np.sqrt(x[0]**2+x[1]**2)
        w = 3*x[2]
        return [gamma*(mu - r**2)*x[0] - w*x[1],
                gamma*(mu - r**2)*x[1] + w*x[0]]
    
    drive = nengo.Ensemble(n_neurons = 2, dimensions = 2, neuron_type=nengo.Direct())
    init_stim = nengo.Node(lambda t : 0.1 if t<0.1 else 0)
    theta = [[0,1],[2,3],[4,5],[6,7],[8,9],[10,11],[12,13],[14,15]]#,[16,17],[18,19],[20,21]] #defines the shape of the CPGs. All neighbors in this array are connected to each other. Make sure to put even numbers on the left and uneven on the right. Also use increasing numbers from top to bottom.
    theta_np = np.array(theta)
    theta_dict = {} # will contain the ensembles, each represents one CPG. The id is the corresponding id in the theta array, the value is the ensemble.
    neurons = 2000
    output = nengo.Ensemble(n_neurons = 16, dimensions = 16, neuron_type=nengo.Direct())
    
    for (x,y), item in np.ndenumerate(theta_np): #creates all CPGs and connects each CPG via the oscillator function to itself.
        item = int(item)
        theta_dict[item] = nengo.Ensemble(n_neurons=neurons, dimensions=4, radius=1.5, label='theta_%d'%item) #3rd represents theta_dot, 1st cos, 2nd sin
        nengo.Connection(theta_dict[item], theta_dict[item][:2], function=oscillator,synapse=0.3) #synaptic delay tau represents the r_dot_dot_i equation
        nengo.Connection(init_stim,theta_dict[item][0])
        if item%2==0:
            nengo.Connection(drive[0],theta_dict[item][2],function=lambda x:high_level_drive_v(x)/3)
            nengo.Connection(drive[0],theta_dict[item][3],function = lambda x:high_level_drive_R(x))
            
            pass
        else:
            nengo.Connection(drive[1],theta_dict[item][2],function = lambda x:high_level_drive_v(x)/3)
            nengo.Connection(drive[1],theta_dict[item][3],function = lambda x:high_level_drive_R(x))
            pass
        nengo.Connection(theta_dict[item][0],output[item])

    # uncomment to record data for offline plot
    #probe_osc = nengo.Probe(output,synapse=0.3)
    def calc_sum_lateral_left(x):
        phi_ij_lateral = -np.pi
        w_ij = 10
        r = np.sqrt(x[0]**2+x[1]**2)
        if r == 0:
            r+= 0.00001
        return 0.33*(w_ij/r * ( ( x[0]*x[3] - x[2]*x[1] )*np.cos(phi_ij_lateral) - ( x[0]*x[2] + x[1]*x[3] )*np.sin(phi_ij_lateral) ))
    
    def calc_sum_lateral_right(x):
        phi_ij_lateral = np.pi+0.01
        w_ij = 10
        r = np.sqrt(x[2]**2+x[3]**2)
        if r == 0:
            r+= 0.00001
        return 0.33*(w_ij/r * ( ( x[2]*x[1] - x[0]*x[3] )*np.cos(phi_ij_lateral) - ( x[2]*x[0] + x[3]*x[1] )*np.sin(phi_ij_lateral) ))
    
    def calc_sum_up(x):
        phi_ij_up = -(np.pi/4)
        w_ij = 10
        r = np.sqrt(x[0]**2+x[1]**2)
        if r == 0:
            r+= 0.00001
        return 0.33*(w_ij/r * ( ( x[0]*x[3] - x[2]*x[1] )*np.cos(phi_ij_up) - ( x[0]*x[2] + x[1]*x[3] )*np.sin(phi_ij_up) ))
   
    def calc_sum_down(x):
        phi_ij_down = (np.pi/4)
        w_ij = 10
        r = np.sqrt(x[2]**2+x[3]**2)
        if r == 0:
            r+= 0.00001
        return 0.33*(w_ij/r * ( ( x[2]*x[1] - x[0]*x[3] )*np.cos(phi_ij_down) - ( x[2]*x[0] + x[3]*x[1] )*np.sin(phi_ij_down) ))

    coupling_dict = {} # contains the ensembles computing the coupling between oscillatory centers
    for n in get_neighbor_pairs(theta_np):
        _n = list(n)
        coupling_dict[n] = nengo.Ensemble(n_neurons=neurons, dimensions=4, radius=2, label='diff_{0}_{1}'.format(*_n))
        #make sure the oscillator with smaller id is written into the first two dimensions
        if _n[0]<_n[1]:
            nengo.Connection(theta_dict[int(_n[0])][:2], coupling_dict[n][:2])
            nengo.Connection(theta_dict[int(_n[1])][:2], coupling_dict[n][2:])
            if _n[0]%2 == _n[1]%2: # up or downward connection
                nengo.Connection(coupling_dict[n], theta_dict[int(_n[0])][2], function=calc_sum_up)   #down
                nengo.Connection(coupling_dict[n], theta_dict[int(_n[1])][2], function=calc_sum_down) #then up
                pass
            else:   #connection of right and left side (contralateral connection)
                nengo.Connection(coupling_dict[n], theta_dict[int(_n[0])][2], function=calc_sum_lateral_left) 
                nengo.Connection(coupling_dict[n], theta_dict[int(_n[1])][2], function=calc_sum_lateral_right)
            pass
        else:
            nengo.Connection(theta_dict[int(_n[1])][:2], coupling_dict[n][:2])
            nengo.Connection(theta_dict[int(_n[0])][:2], coupling_dict[n][2:])
            if _n[0]%2 == _n[1]%2: # up or downward connection
                nengo.Connection(coupling_dict[n], theta_dict[int(_n[1])][2], function=calc_sum_up)   #down
                nengo.Connection(coupling_dict[n], theta_dict[int(_n[0])][2], function=calc_sum_down) #then up
                pass
            else:   #connection of right and left side (contralateral connection)
                nengo.Connection(coupling_dict[n], theta_dict[int(_n[1])][2], function=calc_sum_lateral_left) 
                nengo.Connection(coupling_dict[n], theta_dict[int(_n[0])][2], function=calc_sum_lateral_right)
            pass
    
"""Uncomment to run the neurons in <direct mode (no spiking, just math)"""
""" for ens in model.ensembles:
    ens.neuron_type=nengo.Direct() """

#uncomment to run offline and plot
""" with nengo.Simulator(model) as sim:
    start_time = time.clock()
    sim.run(10)
    print('Time Lasted : {0}'.format(time.clock()-start_time))

plt.plot(sim.trange(), sim.data[probe_osc])
plt.show() """