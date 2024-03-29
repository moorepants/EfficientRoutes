#!/usr/bin/env python

# This example compares two routes connecting Portland, Oregon to Vancouver,
# Washington over a bridge across the Columbia River. The planned bicycle route
# sends the bicyclist through various elevation changes and several yield
# signs, stop signs, and traffic signals where as the automobiles get to travel
# across level ground with no stops. These simulations compare the bicyclist's
# energy expenditure, trip distance, and trip time through the two routes.

import numpy as np
from efficientroutes.model import Bicyclist, Route, Trip

# Load a bicyclist.
bicyclist = Bicyclist()

# Load in the the planned bicycle route data and process the traffic controls
# column.
bicycleRouteData = np.recfromcsv('../data/columbia_river_crossing_bicycle.csv')
stopLocations = []
for i, device in enumerate(bicycleRouteData['traffic_control']):
    if device != '':
        stopLocations.append(bicycleRouteData['distance'][i])
bicycleRoute = Route(bicycleRouteData['distance'],
        bicycleRouteData['elevation'], bicycleRouteData['speed_limit'],
        stopLocations=np.array(stopLocations))

# Setup and compute the results for the trip across the planned bicycle route.
bicycleTrip = Trip(bicyclist, bicycleRoute)
bicycleTrip.solve()
print "===================="
print "Bicycle route stats:"
print "===================="
bicycleTrip.stats()

bicycleFig = bicycleTrip.plot()
bicycleFig.suptitle('Bicycle Route')
bicycleFig.set_figheight(8.0)
bicycleFig.savefig('../data/columbia_river_crossing_bicycle.png', dpi=200)
bicycleFig.show()

# Load in the data for the automobile path.
autoRouteData = np.recfromcsv('../data/columbia_river_crossing_auto.csv')
autoRoute = Route(autoRouteData['distance'],
        autoRouteData['elevation'], autoRouteData['speed_limit'] - 17.88)

# Setup and compute the results for the trip across the automobile route.
autoTrip = Trip(bicyclist, autoRoute)
autoTrip.solve()
print "======================="
print "Automobile route stats:"
print "======================="
autoTrip.stats()

autoFig = autoTrip.plot()
autoFig.suptitle('Automobile Route')
autoFig.set_figheight(8.0)
autoFig.savefig('../data/columbia_river_crossing_auto.png', dpi=200)
autoFig.show()

# Load in the data for the automobile path.
bestRouteData = np.recfromcsv('../data/columbia_river_crossing_best.csv')
stopLocations = []
for i, device in enumerate(bestRouteData['traffic_control']):
    if device != '':
        stopLocations.append(bestRouteData['distance'][i])
bestRoute = Route(bestRouteData['distance'],
        bestRouteData['elevation'], bestRouteData['speed_limit'] - 17.88,
        stopLocations=np.array(stopLocations))

# Setup and compute the results for the trip across the automobile route.
bestTrip = Trip(bicyclist, bestRoute)
bestTrip.solve()
print "================="
print "Best route stats:"
print "================="
bestTrip.stats()

bestFig = bestTrip.plot()
bestFig.suptitle('Best Route')
bestFig.set_figheight(8.0)
bestFig.savefig('../data/columbia_river_crossing_best.png', dpi=200)
bestFig.show()
