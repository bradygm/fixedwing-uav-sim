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
from chap2.mav_viewer import mav_viewer

# initialize the mav viewer
#mav_view = mav_viewer()
mav_view = mav_viewer()

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
while sim_time < SIM.end_time:
    #-------vary states to check viewer-------------

    if sim_time < SIM.end_time/6:
        state.pn += 10*SIM.ts_simulation # 10*sim_time
    elif sim_time < 2*SIM.end_time/6:
        state.pe += 10*SIM.ts_simulation # 10*sim_time
    elif sim_time < 3*SIM.end_time/6:
        state.h += 10*SIM.ts_simulation # 10*sim_time
    elif sim_time < 4*SIM.end_time/6:
        state.phi += .2*SIM.ts_simulation # sim_time #roll
    elif sim_time < 5*SIM.end_time/6:
        state.theta += .2*SIM.ts_simulation #pitch
    else:
        state.psi += .2*SIM.ts_simulation #yaw




    #-------update viewer-------------
    mav_view.update(state)

    #-------increment time-------------
    sim_time += SIM.ts_simulation

print("Press Ctrl-Q to exit...")
pg.QtGui.QApplication.instance().exec_()
