#auke_cpg_benchmark.py
#matplotlib notebook
import nengo
import nengo_loihi
from nxsdk.graph.monitor.probes import PerformanceProbeCondition
from nxsdk.api.n2a import ProbeParameter
import matplotlib.pyplot as plt
import os
import numpy as np
import random
seed = 0
np.random.seed(seed)
random.seed(seed)

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
def high_level_drive_R_left(d):
    d = d[0]
    c_R1 = 0.065    
    c_R0 = 0.196
    R_sat = 0
    d_low = 1.0
    d_high = 5.0
    if d < d_low or d > d_high:
        return [0,0,0,R_sat]
    else:
        return [0,0,0,c_R1*d+c_R0]

def high_level_drive_v_left(d):
    d = d[0]
    c_v1 = 0.2
    c_v0 = 0.3
    v_sat = 0
    d_low = 1.0
    d_high = 5.0
    if d < d_low or d > d_high:
        return [0,0,v_sat/3,0]
    else:
        return [0,0,c_v1*d+c_v0/3,0]
    
def high_level_drive_R_right(d):
    d = d[1]
    c_R1 = 0.065    
    c_R0 = 0.196
    R_sat = 0
    d_low = 1.0
    d_high = 5.0
    if d < d_low or d > d_high:
        return [0,0,0,R_sat]
    else:
        return [0,0,0,c_R1*d+c_R0]

def high_level_drive_v_right(d):
    d = d[1]
    c_v1 = 0.2
    c_v0 = 0.3
    v_sat = 0
    d_low = 1.0
    d_high = 5.0
    if d < d_low or d > d_high:
        return [0,0,v_sat/3,0]
    else:
        return [0,0,c_v1*d+c_v0/3,0]

def oscillator(x):
    if x[0]==0:
        x[0]+=0.01
    #print x[3]
    gamma = 5 #defines how fast it will leave the stable point at [0,0]
    mu = 2*x[3] #the lower this value is the slower the two lateral oscillators diverge
    r = np.sqrt(x[0]**2+x[1]**2)
    w = 3*x[2]
    return [gamma*(mu - r**2)*x[0] - w*x[1],
            gamma*(mu - r**2)*x[1] + w*x[0],
            0,
            0]

def calc_sum_lateral_left(x):
    phi_ij_lateral = np.pi
    w_ij = 10
    r = np.sqrt(x[0]**2+x[1]**2)
    if r == 0:
        r+= 0.00001
    return [0,0,(w_ij/r * ( ( x[0]*x[3] - x[2]*x[1] )*np.cos(phi_ij_lateral) - ( x[0]*x[2] + x[1]*x[3] )*np.sin(phi_ij_lateral) ))/6,0]  #x_i-0;y_i-1;x_j-2;y_j-3  #w_ij*np.sin(diff_angle-phi_ij_lateral)
def calc_sum_lateral_right(x):
    phi_ij_lateral = np.pi
    w_ij = 10
    r = np.sqrt(x[2]**2+x[3]**2)
    if r == 0:
        r+= 0.00001
    return [0,0,(w_ij/r * ( ( x[2]*x[1] - x[0]*x[3] )*np.cos(phi_ij_lateral) - ( x[2]*x[0] + x[3]*x[1] )*np.sin(phi_ij_lateral) ))/6,0]  #x_i-2;y_i-3;x_j-0;y_j-1     #w_ij*np.sin(diff_angle-phi_ij_lateral)
def calc_sum_up(x):
    phi_ij_up = -(np.pi/4)
    w_ij = 10
    r = np.sqrt(x[0]**2+x[1]**2)
    if r == 0:
        r+= 0.00001
    return [0,0,(w_ij/r * ( ( x[0]*x[3] - x[2]*x[1] )*np.cos(phi_ij_up) - ( x[0]*x[2] + x[1]*x[3] )*np.sin(phi_ij_up) ))/6,0]  #x_i-0;y_i-1;x_j-2;y_j-3 #w_ij*np.sin(diff_angle-phi_ij_up)#r_ij*
            #w_ij/r * ( (x_i*y_j - x_j*y_i)*cos(phi) - (x_i*x_j + y_i*y_j)*sin(phi) )
def calc_sum_down(x):
    phi_ij_down = (np.pi/4)
    w_ij = 10
    r = np.sqrt(x[2]**2+x[3]**2)
    if r == 0:
        r+= 0.00001
    return [0,0,(w_ij/r * ( ( x[2]*x[1] - x[0]*x[3] )*np.cos(phi_ij_down) - ( x[2]*x[0] + x[3]*x[1] )*np.sin(phi_ij_down) ))/6,0]  #x_i-2;y_i-3;x_j-0;y_j-1 #w_ij*np.sin(diff_angle-phi_ij_down)#*r_ij
            #w_ij/r * ( (x_i*y_j - x_j*y_i)*cos(phi) - (x_i*x_j + y_i*y_j)*sin(phi) )
    
def dim_01_to_01(x):
    return [x[0],x[1],0,0]

def dim_01_to_23(x):
    return [0,0,x[0],x[1]]

def temp(x):
    return [x[0],0,0,0]

theta = [[0,1],[2,3],[4,5],[6,7],[8,9],[10,11],[12,13],[14,15]]#,[12,13],[14,15]]#,[16,17],[18,19],[20,21]] #defines the shape of the CPGs. All neighbors in this array are connected to each other. Make sure to put even numbers on the left and uneven on the right. Also use increasing numbers from top to bottom.
theta_np = np.array(theta)
theta_dict = {} # will contain the ensembles, each represents one CPG. The id is the corresponding id in the theta array, the value is the ensemble.
diff_dict = {} # contains the ensembles computing the difference betw9een CPGs (theta_j-theta_i)
probe_dict = {}
synapse = 0.01
neurons = 530
model = nengo.Network()
with model:
    #stim = nengo.Node(lambda t: [1.0,1.0] if t < 3 else [5,1])
    stim = nengo.Node([2.5,2.5])
    init_stim = nengo.Node(lambda t : 0.2 if t<0.2 else 0)
    probe = nengo.Probe(stim,'output',synapse=0.0)
    for (x,y), item in np.ndenumerate(theta_np): #creates all CPGs and connects each CPG via the oscillator function to itself.
        item = int(item)
        theta_dict[item] = nengo.Ensemble(n_neurons=neurons, dimensions=4, radius=1.2, label='theta_%d'%item) #3rd represents theta_dot, 1st cos, 2nd sin
        nengo.Connection(theta_dict[item], theta_dict[item], function=oscillator,synapse=0.3) #synaptic delay tau represents the r_dot_dot_i equation
        probe_dict[item] = nengo.Probe(theta_dict[item],synapse=0.01)
        nengo.Connection(init_stim,theta_dict[item],function=temp)
        if item%2==0:
            nengo.Connection(stim,theta_dict[item],function=high_level_drive_v_left)
            nengo.Connection(stim,theta_dict[item],function =high_level_drive_R_left)
        else:
            nengo.Connection(stim,theta_dict[item],function=high_level_drive_v_right)
            nengo.Connection(stim,theta_dict[item],function=high_level_drive_R_right)
    
    # compute the differences between the thetas
    for n in get_neighbor_pairs(theta_np): #creates the diff and phi ensembles, then connects each of those to its corresponding theta ensemble to compute the equation diff_j_i - phi_j_i = (theta_j-theta_i)-phi_i_j
        _n = list(n)
        diff_dict[n] = nengo.Ensemble(n_neurons=neurons, dimensions=4, radius=2, label='diff_{0}_{1}'.format(*_n))
        
            #make sure the oscillator with smaller id is written into the first two dimensions
        if _n[0]<_n[1]:
            nengo.Connection(theta_dict[int(_n[0])], diff_dict[n],synapse=synapse,function=dim_01_to_01)
            nengo.Connection(theta_dict[int(_n[1])], diff_dict[n],synapse=synapse,function=dim_01_to_23)
            if _n[0]%2 == _n[1]%2: # up or downward connection
                print(_n,"vertical connection")
                nengo.Connection(diff_dict[n], theta_dict[int(_n[0])], function=calc_sum_up,synapse=synapse) #down
                nengo.Connection(diff_dict[n], theta_dict[int(_n[1])], function=calc_sum_down,synapse=synapse) #then up
                pass
            else:   #connection of right and left side (contralateral connection)
                print(_n,"lateral connection")
                nengo.Connection(diff_dict[n], theta_dict[int(_n[0])], function=calc_sum_lateral_left,synapse=synapse)
                nengo.Connection(diff_dict[n], theta_dict[int(_n[1])], function=calc_sum_lateral_right,synapse=synapse)
            pass
        else:
            nengo.Connection(theta_dict[int(_n[1])], diff_dict[n],synapse=synapse,function=dim_01_to_01)
            nengo.Connection(theta_dict[int(_n[0])], diff_dict[n],synapse=synapse,function=dim_01_to_23)
            if _n[0]%2 == _n[1]%2: # up or downward connection
                print(_n,"vertical connection")
                nengo.Connection(diff_dict[n], theta_dict[int(_n[1])], function=calc_sum_up,synapse=synapse) #down
                nengo.Connection(diff_dict[n], theta_dict[int(_n[0])], function=calc_sum_down,synapse=synapse) #then up
                pass
            else:   #connection of right and left side (contralateral connection)
                print(_n,"lateral connection")
                nengo.Connection(diff_dict[n], theta_dict[int(_n[1])], function=calc_sum_lateral_left,synapse=synapse)
                nengo.Connection(diff_dict[n], theta_dict[int(_n[0])], function=calc_sum_lateral_right,synapse=synapse)
            pass
n_neurons=model.n_neurons

print("Nr. Neurons",n_neurons)
dt = 0.001
#n_neurons=model.n_neurons
nengo_loihi.set_defaults()
sim = nengo_loihi.Simulator(model,dt=dt)
#board = sim.sims["loihi"].nxsdk_board
#probe_cond = PerformanceProbeCondition(tStart=1, tEnd=int(run_time / dt) * 10, bufferSize=128 * 5, binSize=4)
#t_probe = board.probe(ProbeParameter.EXECUTION_TIME, probe_cond)
#e_probe = board.probe(ProbeParameter.ENERGY, probe_cond)
#import time
#start = time.time()
with sim:
    sim.run(10)
#end= time.time()
#print("Execution Time",end-start)
#print("Execution Time Probe",t_probe.totalExecutionTime)
#print("Energy Probe",e_probe.totalEnergy)
#print("Spiking Energy",e_probe.totalSpikingPhaseEnergy)
#print("Nr. Neurons",n_neurons)
#plt.figure()
#for key,value in probe_dict.items():
#    plt.plot(sim.trange(), sim.data[value][:, 0])
#plt.xlim(0, 10)

fig = plt.figure()
plt.plot(sim.trange(), sim.data[probe])
for key,value in probe_dict.items():
    plt.plot(sim.trange(), sim.data[value][:, 0])
fig.savefig('plot.png')