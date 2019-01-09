"""
mavSimPy 
    - Chapter 2 assignment for Beard & McLain, PUP, 2012
    - Update history:  
        1/9/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM

# load message types
from message_types.msg_state import msg_state
state = msg_state()  # instantiate state message

#from chap2.mav_viewer import mav_viewer
from chap2.spacecraft_viewer import spacecraft_viewer

# initialize the mav viewer
#mav_view = mav_viewer()
mav_view = spacecraft_viewer()

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
while sim_time < SIM.end_time:
    #-------vary states to check viewer-------------
    state.pn = 0 # 10*sim_time
    state.pe = 0 # 10*sim_time
    state.h = 0 # 10*sim_time
    state.phi = 0 # sim_time
    state.theta = 0 # sim_time
    state.psi = sim_time

    #-------update viewer-------------
    mav_view.update(state)

    #-------increment time-------------
    sim_time += SIM.ts_simulation

print("Press Ctrl-Q to exit...")
pg.QtGui.QApplication.instance().exec_()




