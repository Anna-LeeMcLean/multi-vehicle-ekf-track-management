# Multi Target Track Management

This project tracks multiple vehicles using an Extended Kalman Filter (EKF) and various track management concepts. The EKF performs sensor fusion of LiDAR and camera measurements to estimate the position and velocity of detected obstacles in the environment. Fusing is done by performing the EKF update step twice for each prediction, once with a LiDAR measurement then again with a camera measurement. LiDAR and camera data from the Waymo open source dataset is used. 

Video

If you want to clone and use this repo, please email me for the data and Waymo tools needed to parse the data which are required to run the code.

## Track Management
Each obstacle detected in the environment is assigned a 'track', which is an object that consists of a position+velocity estimate of the obstacle, a relevance score and a state. Every time a track is updated with a measurement, the track’s relevance score is increased, otherwise it is decreased. Any track whose score decreases below a specified threshold is deleted. This ensures that only the obstacles which are being seen continuously are tracked. A track’s state can be either initialized, tentative or confirmed. The track’s score triggers transitions between these states using threshold values. Every new track, which represents a new obstacle that has come into the car's field of view, is created in the initialized state. The tentative state is introduced to make sure that we don't track false positive obstacles. False positives won't have continuous measurements assigned to them, and would therefore be deleted before they can reach the confirmed state. Only objects with confirmed tracks are considered as obstacles which the ego car should avoid.

Because multiple obstacles are being tracked simultaneously, a system is needed to:
1. Associate sensor measurements with their correct tracks.
2. Create a new track whenever there is an unassigned measurement.
3. Delete old tracks whenever they become irrelevant to the ego car.



The position estimate of each track is predicted to the next time step with the EKF, then associated with its sensor measurements, then updated with the EKF again.
such ass data association, gating and 
