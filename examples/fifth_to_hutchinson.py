#!/usr/bin/env python

# This compares three routes from 5th and L to Hutchinson and California in
# Davis, California. The routes are via third, fifth, eighth streets.

import numpy as np
from efficientroutes.model import Bicyclist, Route, Trip

bicyclist = Bicyclist()

routes = {}
for way in ['third', 'fifth', 'eighth']:
    routeData = np.recfromcsv('../data/fifth_to_hutchinson_via_' + way + '.csv')

    stopLocations = []
    for i, device in enumerate(routeData['traffic_control']):
        if device != '':
            stopLocations.append(routeData['distance'][i])

    routes[way] = Route(routeData['distance'],
            routeData['elevation'], routeData['speed_limit'],
            stopLocations=np.array(stopLocations))

for k, v in routes.items():
    trip = Trip(bicyclist, v)
    trip.solve()
    string = '{} route stats:'.format(k.capitalize())
    print "=" * len(string)
    print string
    print "=" * len(string)
    trip.stats()

    fig = trip.plot()
    fig.suptitle('Via {}'.format(k.capitalize()))
    fig.set_figheight(8.0)
    fig.savefig('../data/fith-to-hutchinson-via-' + k + '.png', dpi=200)
    fig.show()
