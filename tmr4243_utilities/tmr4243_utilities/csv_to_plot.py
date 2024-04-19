import pandas as pd

import matplotlib.pyplot as plt

# Read the CSV file
data = pd.read_csv('home/andeshog/sim_ws/bags/ObserverRecording1/_CSEI_state_eta.csv')

# Extract the columns you want to plot
x = data['Timestamp']
y = data['eta.1']

# Plot the data
plt.plot(x, y)
plt.xlabel('X-axis label')
plt.ylabel('Y-axis label')
plt.title('Plot Title')
plt.grid(True)
plt.show()