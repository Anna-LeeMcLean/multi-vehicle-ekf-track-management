# Multi Target Track Management

This project tracks multiple vehicles using an Extended Kalman Filter (EKF) and various track management concepts. The EKF performs sensor fusion of LiDAR and camera measurements to estimate the position and velocity of detected obstacles in the environment. Fusing is done by performing the EKF update step twice for each prediction, once with a LiDAR measurement then again with a camera measurement. LiDAR and camera data from the Waymo open source dataset is used. 



If you want to clone and use this repo, please email me for the data and Waymo tools required to run the code.
Email: mcleanannalee@gmail.com

## Track Management
Each obstacle detected in the environment is assigned a 'track', which is an object that consists of a position+velocity estimate of the obstacle, a relevance score and a state. Every time a track is updated with a measurement, the track’s relevance score is increased, otherwise it is decreased. Any track whose score decreases below a specified threshold is deleted. This ensures that only the obstacles which are being seen continuously are tracked. A track’s state can be either initialized, tentative or confirmed. The track’s score triggers transitions between these states using threshold values. Every new track, which represents a new obstacle that has come into the car's field of view, is created in the initialized state. The tentative state is introduced to make sure that we don't track false positive obstacles. False positives won't have continuous measurements assigned to them, and would therefore be deleted before they can reach the confirmed state. Only objects with confirmed tracks are considered as obstacles which the ego car should avoid.

## Data Association
Because multiple obstacles are being tracked simultaneously, a system is needed to associate sensor measurements with their correct tracks. A simple nearest neighbour approach determined which measurements were closest to which tracks. The closest measurement to a track was assigned to that track and used to update the track’s prediction. The Malahanobis distance was used to determine the proximity of measurements to tracks so that the covariance of the tracks’ estimates would be considered. Gating simplified this process by ignoring measurements that were too far away any tracks. This distance was calculated for every track and measurement and stored in an association matrix. 

<img width="1536" height="760" alt="rmse step 4 - multi target tracking and sensor fusion" src="https://github.com/user-attachments/assets/7c03d259-ff38-441e-905f-06bdac011e91" />
RMSE for confirmed tracks
