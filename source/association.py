# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Data association class with single nearest neighbor association and gating based on Mahalanobis distance
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
from scipy.stats.distributions import chi2

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import misc.params as params 

class Association:
    '''Data association class with single nearest neighbor association and gating based on Mahalanobis distance'''
    def __init__(self):
        self.association_matrix = np.matrix([])
        self.unassigned_tracks = []
        self.unassigned_meas = []
        
    def associate(self, track_list, meas_list, KF):
             
        ############
        # TODO Step 3: association:
        # - replace association_matrix with the actual association matrix based on Mahalanobis distance (see below) for all tracks and all measurements
        # - update list of unassigned measurements and unassigned tracks
        ############
        N = len(track_list) # N tracks
        M = len(meas_list) # M measurements
        
        self.association_matrix = np.inf*np.ones((N,M)) 

        for i,track in enumerate(track_list):
            for j,meas in enumerate(meas_list):
                mhd = self.MHD(track, meas, KF)
                if self.gating(mhd, meas.sensor):
                    self.association_matrix[i,j] = mhd

        # print(f"{self.association_matrix = }")
        
        # reset lists
        # NOTE: the tracks and measurements that are assigned will be removed from this list in the get_closest_track_and_meas() function
        self.unassigned_tracks = list(range(N))
        self.unassigned_meas = list(range(M))

        # -----------------------------------------------------------------------------------

        # NOTE: Below is my idea for this. Need to update the get_closest_track_and_meas()
        # function if you want to try it out

        # if a column has all infinities, this measurement has not been assigned to a track
        # if a row has all infinities, that track has no measurement

        # rows, cols = self.association_matrix.shape
        # inf_matrix = np.full_like(self.association_matrix, np.inf)

        # # determine unassigned measurements
        # for i in range(cols):
        #     col = self.association_matrix[:,i]
        #     col_inf = inf_matrix[:,i]
        #     if np.array_equal(col, col_inf):
        #         self.unassigned_meas.append(i)

        # # determine unassigned tracks
        # for i in range(rows):
        #     row = self.association_matrix[i,:]
        #     row_inf = inf_matrix[i,:]
        #     if np.array_equal(row, row_inf):
        #         self.unassigned_tracks.append(i)
        
        ############
        # END student code
        ############ 
                
    def get_closest_track_and_meas(self):
        ############
        # TODO Step 3: find closest track and measurement:
        # - find minimum entry in association matrix
        # - delete row and column
        # - remove corresponding track and measurement from unassigned_tracks and unassigned_meas
        # - return this track and measurement
        ############

        A = self.association_matrix
        if np.min(A) == np.inf:
            return np.nan, np.nan
        
        idx = np.unravel_index(np.argmin(A, axis=None), A.shape)    # returns the row and col of the smallest element in array
        A = np.delete(A, idx[0], 0) # delete row
        A = np.delete(A, idx[1], 1) # delete column
        self.association_matrix = A

        # the following only works for at most one track and one measurement
        update_track = self.unassigned_tracks[idx[0]] 
        update_meas = self.unassigned_meas[idx[1]]
        
        # remove from list
        self.unassigned_tracks.remove(update_track) 
        self.unassigned_meas.remove(update_meas)
            
        ############
        # END student code
        ############ 
        return update_track, update_meas     

    def gating(self, MHD, sensor): 
        ############
        # TODO Step 3: return True if measurement lies inside gate, otherwise False
        ############
        
        limit = chi2.ppf(params.gating_threshold, df=sensor.dim_meas)
        
        return MHD < limit    
        
        ############
        # END student code
        ############ 
        
    def MHD(self, track, meas, KF):
        ############
        # TODO Step 3: calculate and return Mahalanobis distance
        ############
        
        H = meas.sensor.get_H(track.x)
        resid = KF.gamma(track, meas)
        S = KF.S(track, meas, H)

        return resid.T * S.I * resid
        
        ############
        # END student code
        ############ 
    
    def associate_and_update(self, manager, meas_list, KF):
        # associate measurements and tracks
        self.associate(manager.track_list, meas_list, KF)
    
        # update associated tracks with measurements
        while self.association_matrix.shape[0]>0 and self.association_matrix.shape[1]>0:
            
            # search for next association between a track and a measurement
            ind_track, ind_meas = self.get_closest_track_and_meas()
            if np.isnan(ind_track):
                print('---no more associations---')
                break
            track = manager.track_list[ind_track]
            
            # check visibility, only update tracks in fov    
            if not meas_list[0].sensor.in_fov(track.x):
                continue
            
            # Kalman update
            print('update track', track.id, 'with', meas_list[ind_meas].sensor.name, 'measurement', ind_meas)
            KF.update(track, meas_list[ind_meas])
            
            # update score and track state 
            manager.handle_updated_track(track)
            
            # save updated track
            manager.track_list[ind_track] = track
            
        # run track management 
        manager.manage_tracks(self.unassigned_tracks, self.unassigned_meas, meas_list)
        
        for track in manager.track_list:  
            print('track: id =', track.id, 'state =' , track.state, 'score =', track.score)