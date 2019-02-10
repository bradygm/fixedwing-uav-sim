import sys
sys.path.append('..')
import numpy as np

######################################################################################
                #   sample times, etc
######################################################################################
ts_simulation = 0.02  # smallest time step for simulation
start_time = 0.  # start time for simulation
end_time = 60.  # end time for simulation

ts_plotting = 0.5  # refresh rate for plots

ts_control = ts_simulation  # sample rate for the controller

#Wind parameters
Lu = 200
Lv = Lu
Lw = 50
sigma_u = 1.06
sigma_v = sigma_u
sigma_w = .7
