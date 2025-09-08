import matplotlib.pyplot as plt


incline_10_deg = [5.83, 9.15, 11.97]
incline_20_deg = [23.87, 20.52, 20.87]
incline_30_deg = [22.89, 26.48, 27.65]

heading_10_deg = [97.41, 84.1, 85.09]
heading_20_deg = [272.77, 266.74, 272.73]
heading_30_deg = [88.88, 93.7, 91.61]

truth_incline = [11.3, 20.6, 30.5]
truth_heading = [90.0, 270.0, 90.0]

# Example for incline
fig, ax = plt.subplots()
x_labels = [10, 20, 30]

marker_size = 120
ax.scatter([truth_incline[0] for x in range(len(incline_10_deg))], incline_10_deg, s=marker_size, color='blue', label='11.3 deg incline')
ax.scatter([truth_incline[1] for x in range(len(incline_20_deg))], incline_20_deg, s=marker_size, color='green', label='20.6 deg incline')
ax.scatter([truth_incline[2] for x in range(len(incline_30_deg))], incline_30_deg, s=marker_size, color='red', label='30.5 deg incline')

# Plot truth values
ax.plot(truth_incline, truth_incline, linewidth=4, color='black', linestyle='--', label='Truth')

fontsize = 22
ax.set_xlabel('Incline [deg]', fontsize=fontsize)
ax.set_ylabel('Measured Incline [deg]', fontsize=fontsize)
ax.set_xticks(truth_incline)
plt.xticks(fontsize=fontsize)
plt.yticks(fontsize=fontsize)
plt.xlim([7, 33])
plt.ylim([3, 32])
ax.legend(fontsize=fontsize, loc='lower right')
ax.grid(True)
plt.show()