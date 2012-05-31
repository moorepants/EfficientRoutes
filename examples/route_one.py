import numpy as np
import matplotlib.pyplot as plt

from efficientroutes import model

bicyclist = model.Bicyclist()

data = np.recfromcsv('../data/test_route.csv')
# get stop locations (for now yield, stop lights, and stop signs are treated
# the same)
stopLocations = []
for i, device in enumerate(data['traffic_control']):
    if device != '':
        stopLocations.append(data['distance'][i])

route = model.Route(data['distance'], data['elevation'], data['speed_limit'],
        stopLocations=np.array(stopLocations))
fig = route.plot()

trip = model.Trip(bicyclist, route)
trip.solve()
fig2 = trip.plot()

fig.axes[0].plot(trip.rootDistances, np.interp(trip.rootDistances,
    trip.route.distance, trip.route.elevation), 'vy')

fig.show()
fig2.show()
