import matplotlib.pyplot as plt

# data 1
path_length = [100, 200, 300, 400, 500, 600, 700, 800]
# note that we need to carry out runs for each of these length multiple times
# which will lead us to following results

## TRANSLATION ERROR W.R.TO PATH LENGTH
te_per_l= [0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800] # random values for now
## ROTATION ERROR W.R.TO PATH LENGTH
re_l = [0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800] # deg/m

# data 2
speed = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
# note that we need to carry out runs for each of these speeds multiple times
# which will lead us to following results

## TRANSLATION ERROR W.R.TO SPEED
te_per_s= [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0] # random values for now
## ROTATION ERROR W.R.TO SPEED
re_s = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0] # deg/m

# data 3
estimated_traj_x = [0, 1.1, 1.9, 3.1, 4.2, 5.05, 6.1, 7.1, 8.1, 9.1]
estimated_traj_y = [0, 1.1, 3.9, 6.7, 14.5, 24.3, 35.6, 49.2, 64.1, 81.2]
gt_x = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
gt_y = [0, 1, 4, 9, 16, 25, 36, 49, 64, 81]

# We need three figs
fig1, axs1 = plt.subplots(1) # for trajectory
fig2, axs2 = plt.subplots(2, sharex = True) # for path length
fig3, axs3 = plt.subplots(2, sharex = True) # for speed

# TRAJECTORY PLOT
axs1.plot(estimated_traj_x, estimated_traj_y, 'b-', label = "Laser Odometry")
axs1.plot(gt_x, gt_y, 'r-', label = "Ground Truth")
axs1.plot(gt_x[0], gt_y[0], 'ko', label = "Sequence Start")
axs1.legend()

# PATH LENGTH PLOT
axs2[0].plot(path_length, te_per_l, 'bo-', label = "Translation Error")
# axs2[0].set_xlabel("Path Length [m]")
axs2[0].set_ylabel("Translation Error [%]")
axs2[1].plot(path_length, re_l, 'bo-', label = "Rotation Error")
axs2[1].set_xlabel("Path Length [m]")
axs2[1].set_ylabel("Rotation Error [deg/m]")

# SPEED PLOT
axs3[0].plot(speed, te_per_s, 'bo-', label = "Translation Error")
axs3[0].set_ylabel("Translation Error [%]")
axs3[1].plot(speed, re_s, 'bo-', label = "Rotation Error")
axs3[1].set_xlabel("Speed [m/s]")
axs3[1].set_ylabel("Rotation Error [deg/m]")
plt.show()
