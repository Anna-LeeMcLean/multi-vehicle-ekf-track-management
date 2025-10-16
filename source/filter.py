# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass

    def F(self):
        ############
        # Step 1: implement and return system matrix F
        ############

        dt = params.dt
        return np.matrix([[1, 0, 0, dt, 0, 0],
                          [0, 1, 0, 0, dt, 0],
                          [0, 0, 1, 0, 0, dt],
                          [0, 0, 0, 1, 0, 0 ], 
                          [0, 0, 0, 0, 1, 0 ],
                          [0, 0, 0, 0, 0, 1 ]])

    def Q(self):
        ############
        # Step 1: implement and return process noise covariance Q
        ############
        dt = params.dt
        q  = params.q
        
        return np.matrix([[1/3*dt**3*q,       0,           0,      1/2*dt**2*q,      0,          0      ],
                          [     0,       1/3*dt**3*q,      0,           0,      1/2*dt**2*q,     0      ],
                          [     0,            0,      1/3*dt**3*q,      0,           0,      1/2*dt**2*q],
                          [1/2*dt**2*q,       0,           0,         dt*q,          0,          0      ],
                          [     0,       1/2*dt**2*q,      0,           0,         dt*q,         0      ],
                          [     0,            0,       1/2*dt**2*q,     0,           0,        dt*q  ]])

    def predict(self, track):
        ############
        # Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############

        F = self.F()
        x = F*track.x # state prediction
        P = F*track.P*F.transpose() + self.Q() # covariance prediction

        track.set_x(x)
        track.set_P(P)
        return x, P 

    def update(self, track, meas):
        ############
        # Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        
        H = meas.sensor.get_H(track.x)
        gamma = self.gamma(track, meas)
        S = self.S(track, meas, H)
        
        K = track.P*H.T*S.I # Kalman gain
        x = track.x + K*gamma # state update
        I = np.identity(params.dim_state)
        P = (I - K*H) * track.P # covariance update

        track.set_x(x)
        track.set_P(P)
        
        track.update_attributes(meas)

        return x, P 
    
    def gamma(self, track, meas):
        ############
        #  Step 1: calculate and return residual gamma
        ############

        x_meas = meas.sensor.get_hx(track.x)
        return meas.z - x_meas

    def S(self, track, meas, H):
        ############
        #  Step 1: calculate and return covariance of residual S
        ############

        return H*track.P*H.T + meas.R