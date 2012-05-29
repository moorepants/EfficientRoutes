# /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import sundials

# debugging
#try:
    #from IPython.core.debugger import Tracer
#except ImportError:
    #pass
#else:
    #set_trace = Tracer()

class Base(object):
    def update_attributes(self, default, **kwargs):
        for k, v in default.items():
            if k in kwargs.keys():
                setattr(self, k, kwargs[k])
            else:
                setattr(self, k, v)

class Bicyclist(Base):

    def __init__(self, **kwargs):
        """Instantiates a Bicyclist object.

        Parameters
        ----------
        mass : float, optional, default=80.0
            Total mass of the bicycle and the rider in kilograms.
        dragCoefficient : float, optional, default=1.15

        """
        default =  {'mass': 80.0, # kg
                    'dragCoefficient' : 1.15,
                    'frontalArea' : 0.55, # square meters
                    'maxPower' : 100, # watts
                    'brakingTime': 0.1, # seconds
                    'ridingStyle' : 'normal',
                    }

        self.update_attributes(default, **kwargs)

    def weight(self):
        """Returns the bicyclist and bicycle's weight."""
        g = 9.81
        return self.mass * g

    def aero_drag(self, airDensity, speed):
        """Returns the aerodynamics drag force.

        Parameters
        ----------
        airDensity : float
            The density of air in kg/m**3.

        """
        dynamicPressure = airDensity * speed**2 / 2.0
        return (-np.sign(speed) * self.dragCoefficient * dynamicPressure *
                self.frontalArea)

    def normal_force(self, angle):
        """Returns the normal force acting on the bicyclist.

        Parameters
        ----------
        angle : float
            The angle in radians between the direction of travel and the
            ground.

        """
        return self.weight() * np.cos(angle)

    def incline_force(self, angle):
        """Returns the force due to gravity which resists or increases the
        bicyclist's forward motion.

        Parameters
        ----------
        angle : float
            The angle in radians between the direction of travel and the
            ground.

        """
        return -self.weight() * np.sin(angle)

    def rolling_friction(self, rollingCoefficient, speed, angle):
        """Returns the force due to rolling resistance."""
        return (-np.sign(speed) * rollingCoefficient * self.normal_force(angle) *
            (1.0 - np.exp(-speed / 10.0)))

    def max_brake_force(self, adhesionCoefficient, angle):
        """Returns the maximum braking force."""
        return adhesionCoefficient * self.normal_force(angle)

    def max_propulsion(self, speed, adhesionCoefficient, angle):
        """Returns the minimum of the rider's maximum propulsive force and the
        maximum braking force."""
        return min(self.maxPower / speed, self.max_brake_force(adhesionCoefficient, angle))

class Route(Base):

    def __init__(self, distance, elevation, speedLimit, stopSignLocations=None,
            trafficLights=None, **kwargs):
        """

        Parameters
        ----------
        distance : array_like
            The distance traveled along the route at equidistant points.
        elevation : array_like
            The elevation of the route at each point in `distance`.
        speedLimit : array_like
            The speed limit at each point in `distance`.

        """
        default =  {'rollingFrictionCoefficient': 0.007,
                    'tireAdhesionCoefficient': 0.17,
                    'airDensity': 1.2,
                    }
        self.update_attributes(default, **kwargs)

        self.distance = distance
        self.elevation = elevation
        self.speedLimit = speedLimit

    def slope(self):
        """Returns the slope of the route."""
        return np.hstack((0, np.diff(self.elevation) / np.diff(self.distance)))

    def angle(self):
        """Returns the angle of the slope in radians."""
        return np.arctan(self.slope())

    def brake_start(self):
        """Returns the locations at which the rider should start braking for
        each stop sign."""
        brakingDistance =  (self.speedLimit**2 / (20.0 *
            (self.rollingFrictionCoefficient + self.tireAdhesionCoefficient +
                np.sin(self.slope().min()))))
        brakingDistanceAtStop = np.interp1(self.distance, brakingDistance,
                self.stopSignLocations)
        return self.stopSignLocations - brakingDistanceAtStop

    def plot(self):
        """Returns a plot of the distance versus elevation."""

        fig = plt.figure()

        ax = fig.add_subplot(2, 1, 1)
        ax.plot(self.distance, self.elevation)
        ax.set_xlabel('Distance [m]')
        ax.set_ylabel('Height [m]')

        ax = fig.add_subplot(2, 1, 2)
        ax.plot(self.distance, self.slope())
        ax.set_xlabel('Distance [m]')
        ax.set_ylabel('Slope [m/m]')

        return fig

class Trip(object):

    def __init__(self, bicyclist, route):
        self.bicyclist = bicyclist
        self.route = route

    def rhs(self, t, states):
        """The right hand side of the differential equations.

        Parameters
        ----------
        t : float
            Time.
        states : array_like, shape(3,)
            States: [distance, speed, energy]
        switch : list
            The switch boolean values.

        Returns
        -------
        dstates : list
            The derivatives of the states [speed, acceleration, power].

        """
        distance = states[0]
        speed = states[1]
        energy = states[2]

        dstates = [0., 0., 0.]

        # speed
        dstates[0] = states[1]

        # acceleration
        angle = np.interp(distance, self.route.distance, self.route.angle())
        Fp = self.bicyclist.max_propulsion(speed, self.route.tireAdhesionCoefficient, angle)
        Fd = self.bicyclist.aero_drag(speed, self.route.airDensity)
        Fi = self.bicyclist.incline_force(angle)
        Fr = self.bicyclist.rolling_friction(self.route.rollingFrictionCoefficient, speed, angle)
        Fb = 0.0
        forces = Fp + Fd + Fi + Fr + Fb
        dstates[1] = forces / self.bicyclist.mass

        # power
        dstates[2] = Fp / (speed)

        return dstates

    def event(self, ):
        pass

    def solve(self, ):
        # Setup the solver.
        solver = sundials.CVodeSolver(RHS=self.rhs, ROOT=None, SW=None,
                       abstol = 1.0e-6, reltol = 1.0e-6)

        # The initial conditions
        t0 = 0.0
        y0 = [0., self.route.speedLimit[0], 0.]
        solver.init(t0, y0)

        dt = 1. # integration step size
        iterate = solver.iter(t0, dt)
        # Initialize lists to collect the time and the height of the ball.
        time = [t0]
        stateHistory = [y0]
        while True:
            t, y = next(iterate)
            time.append(t)
            stateHistory.append([y[0], y[1], y[2]])

            if y[0] > self.route.distance[-1]:
                break

        return np.array(time), np.array(stateHistory)
