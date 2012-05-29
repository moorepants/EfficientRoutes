import numpy as np
import matplotlib.pyplot as plt

import model

bicyclist = model.Bicyclist()

data = np.recfromcsv('../data/test_route.csv')
route = model.Route(data['distance'], data['elevation'], data['speed_limit'])
fig = route.plot()

trip = model.Trip(bicyclist, route)

time, states = trip.solve()

fig2 = plt.figure()

ax = fig2.add_subplot(4, 1, 1)
line = ax.plot(time, np.interp(states[:, 0], route.distance, route.elevation),
        label='Elevation [m]')
ax.set_ylabel(line[0].get_label())

ax = fig2.add_subplot(4, 1, 2)
line = ax.plot(time, states[:, 0], label='Distance [m]')
ax.set_ylabel(line[0].get_label())

ax = fig2.add_subplot(4, 1, 3)
line = ax.plot(time, states[:, 1], label='Speed [m/s]')
ax.set_ylabel(line[0].get_label())

ax = fig2.add_subplot(4, 1, 4)
line = ax.plot(time, states[:, 2], label='Energy [J]')
ax.set_ylabel(line[0].get_label())
ax.set_xlabel('Time [s]')

fig2.show()
