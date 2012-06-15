#!/usr/bin/env python

# This is a basic comparison of simple trips.

import numpy as np
from efficientroutes.model import Bicyclist, Route, Trip

bicyclist = Bicyclist()

flatNoStopData = np.recfromcsv('../data/flat_no_stops.csv')
stopLocations = []
for i, device in enumerate(flatNoStopData['traffic_control']):
    if device != '' and device != False:
        stopLocations.append(flatNoStopData['distance'][i])
flatNoStopRoute = Route(flatNoStopData['distance'],
        flatNoStopData['elevation'], flatNoStopData['speed_limit'],
        stopLocations=np.array(stopLocations))

flatNoStopTrip = Trip(bicyclist, flatNoStopRoute)
flatNoStopTrip.solve()
print "==============================="
print "Flat with no stops route stats:"
print "==============================="
flatNoStopTrip.stats()

flatNoStopFig = flatNoStopTrip.plot()
flatNoStopFig.suptitle('Flat, No Stops')
flatNoStopFig.set_figheight(8.0)
flatNoStopFig.savefig('../data/flat-no-stops.png', dpi=200)
flatNoStopFig.show()

flatWithStopData = np.recfromcsv('../data/flat_with_stops.csv')
stopLocations = []
for i, device in enumerate(flatWithStopData['traffic_control']):
    if device != '' and device != False:
        stopLocations.append(flatWithStopData['distance'][i])
flatWithStopRoute = Route(flatWithStopData['distance'],
        flatWithStopData['elevation'], flatWithStopData['speed_limit'],
        stopLocations=np.array(stopLocations))

flatWithStopTrip = Trip(bicyclist, flatWithStopRoute)
flatWithStopTrip.solve()
print "============================"
print "Flat with stops route stats:"
print "============================"
flatWithStopTrip.stats()

flatWithStopFig = flatWithStopTrip.plot()
flatWithStopFig.suptitle('Flat, With Stops')
flatWithStopFig.set_figheight(8.0)
flatWithStopFig.savefig('../data/flat-with-stops.png', dpi=200)
flatWithStopFig.show()

hillyNoStopData = np.recfromcsv('../data/hilly_no_stops.csv')
stopLocations = []
for i, device in enumerate(hillyNoStopData['traffic_control']):
    if device != '' and device != False:
        stopLocations.append(hillyNoStopData['distance'][i])
hillyNoStopRoute = Route(hillyNoStopData['distance'],
        hillyNoStopData['elevation'], hillyNoStopData['speed_limit'],
        stopLocations=np.array(stopLocations))

hillyNoStopTrip = Trip(bicyclist, hillyNoStopRoute)
hillyNoStopTrip.solve()
print "================================"
print "Hilly with no stops route stats:"
print "================================"
hillyNoStopTrip.stats()

hillyNoStopFig = hillyNoStopTrip.plot()
hillyNoStopFig.suptitle('Hilly, No Stops')
hillyNoStopFig.set_figheight(8.0)
hillyNoStopFig.savefig('../data/hilly-no-stops.png', dpi=200)
hillyNoStopFig.show()

hillyWithStopsData = np.recfromcsv('../data/hilly_with_stops.csv')
stopLocations = []
for i, device in enumerate(hillyWithStopsData['traffic_control']):
    if device != '' and device != False:
        stopLocations.append(hillyWithStopsData['distance'][i])
hillyWithStopsRoute = Route(hillyWithStopsData['distance'],
        hillyWithStopsData['elevation'], hillyWithStopsData['speed_limit'],
        stopLocations=np.array(stopLocations))

hillyWithStopsTrip = Trip(bicyclist, hillyWithStopsRoute)
hillyWithStopsTrip.solve()
print "============================="
print "Hilly with stops route stats:"
print "============================="
hillyWithStopsTrip.stats()

hillyWithStopsFig = hillyWithStopsTrip.plot()
hillyWithStopsFig.suptitle('Hilly, With Stops')
hillyWithStopsFig.set_figheight(8.0)
hillyWithStopsFig.savefig('../data/hilly-with-stops.png', dpi=200)
hillyWithStopsFig.show()
