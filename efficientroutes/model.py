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
            The drag coefficient.
        frontalArea : float, optional, default=0.55
            The frontal area of the bicyclist and bicycle in square meters.
        maxPower : float, optional, default=100.0
            The maximum power in watts the bicyclist will produce during a trip.
        brakingTime : float, optional, default=0.1
            The time in seconds in which the rider should reduce speed if going
            above the speed limit.


        """
        default =  {'mass': 80.0, # kg
                    'dragCoefficient' : 1.15,
                    'frontalArea' : 0.55, # square meters
                    'maxPower' : 100.0, # watts
                    'brakingTime': 0.1, # seconds
                    }

        self.update_attributes(default, **kwargs)

    def _check_angle(self, angle):
        """Raises an error if the angle is not between -pi/2 and pi/2."""
        if angle < -np.pi / 2.0 or angle > np.pi / 2.0:
            raise ValueError('The angle {} is not between -pi/2 and pi/\
                    2.'.format(angle))


    def weight(self):
        """Returns the combined weight of the bicyclist and bicycle in Newtons."""
        g = 9.81
        return self.mass * g

    def aero_drag(self, airDensity, speed):
        """Returns the aerodynamics drag force.

        Parameters
        ----------
        airDensity : float
            The density of air in kg/m**3.
        speed : float
            The speed of the bicyclist.

        Returns
        -------
        drag : float
            The drag force which is always opposite sign as the speed.

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

        Returns
        -------
        normal : float
            The force in newtons applied from the ground to the bicyclist.

        """
        self._check_angle(angle)
        return self.weight() * np.cos(angle)

    def incline_force(self, angle):
        """Returns the force due to gravity which resists or increases the
        bicyclist's forward motion.

        Parameters
        ----------
        angle : float
            The angle in radians between the direction of travel and the
            ground.

        Returns
        -------
        force : float
            The component of force acting in the direction of the angle. A
            positive angle returns a negative force and vice versa.

        """
        self._check_angle(angle)
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
        return min(self.maxPower /
                speed, self.max_brake_force(adhesionCoefficient, angle))

    def stop_brake_force(self, speed, desiredSpeed):
        """Returns the braking force needed to decelerate in given braking
        time to the desired speed."""
        return -self.mass * (speed - desiredSpeed) / self.brakingTime

    def distance_to_stop(self, speed, adhesionCoefficient, angle):
        """Returns the estimated distance to stop neglecting aerodynamics drag
        and rolling friction."""
        return (self.mass * speed**2 / 2 / abs(self.incline_force(angle) -
                self.max_brake_force(adhesionCoefficient, angle)))

class Route(Base):

    def __init__(self, distance, elevation, speedLimit, stopLocations=None,
            **kwargs):
        """Instantiates a Route object with the provided data.

        Parameters
        ----------
        distance : array_like, shape(n,)
            The distance traveled along the route at equidistant points.
        elevation : array_like, shape(n,)
            The elevation of the route at each point in `distance`.
        speedLimit : array_like, shape(n,)
            The speed limit at each point in `distance`.
        stopSignLocations : array_like, shape(m,)
            The distances at which their are stop signs.
        rollingFrictionCoefficient : float, default=0.007, optional
            The rolling friction coefficient.
        adhesionCoefficient : float, default=0.17, optional
        airDensity : float, default=1.2, optional

        """
        default =  {'rollingFrictionCoefficient': 0.007,
                    'adhesionCoefficient': 0.17,
                    'airDensity': 1.2,
                    }
        self.update_attributes(default, **kwargs)

        self.distance = distance
        self.elevation = elevation
        self.speedLimit = speedLimit
        if stopLocations is None:
            self.stopLocations = None
        else:
            self.stopLocations = stopLocations

    def slope(self):
        """Returns the slope of the route."""
        return np.hstack((0, np.diff(self.elevation) / np.diff(self.distance)))

    def angle(self):
        """Returns the angle of the slope in radians."""
        return np.arctan(self.slope())

    def current_angle(self, distance):
        """Returns the angle of the slope in radians at the provided location."""
        return np.interp(distance, self.distance, self.angle())

    def current_speed_limit(self, distance):
        """Returns the speed limit in meters per second at the provided location."""
        return np.interp(distance, self.distance, self.speedLimit)

    def next_stop(self, distance):
        """Returns the next stop location in meters from the start.

        Parameters
        ----------
        distance : float
            The distance to check for stops.

        Returns
        -------
        nextStop : float | None
            The location of the next stop. Returns None if there are no
            remaining stops.

        """
        if self.stopLocations is None:
            return None
        else:
            try:
                nextStop = self.stopLocations[self.stopLocations > distance][0]
            except IndexError:
                nextStop = np.inf

            return nextStop

    def distance_to_stop(self, distance):
        """Returns the distance in meters to the next stop."""
        nextStop = self.next_stop(distance)
        if nextStop is None:
            distanceToStop = None
        else:
            distanceToStop = nextStop - distance

        return distanceToStop

    def plot(self):
        """Returns a plot of the distance versus elevation."""

        fig = plt.figure()

        ax = fig.add_subplot(2, 1, 1)
        ax.plot(self.distance, self.elevation)
        ax.set_xlabel('Distance [m]')
        ax.set_ylabel('Height [m]')

        ax2 = fig.add_subplot(2, 1, 2, sharex=ax)
        ax2.plot(self.distance, self.slope())
        ax2.set_xlabel('Distance [m]')
        ax2.set_ylabel('Slope [m/m]')

        if self.stopLocations is not None:
            ax.plot(self.stopLocations, np.interp(self.stopLocations,
                self.distance, self.elevation), 'or')

        return fig

class Trip(object):

    def __init__(self, bicyclist, route):
        """Instantiates a Trip object.

        Parameters
        ----------
        bicyclist : Bicyclist
            The bicyclist for the trip.
        route : Route
            The route for the trip.

        """
        self.bicyclist = bicyclist
        self.route = route

    def create_stop_list(self):
        """Creates a temporary list of the stop locations for use in the
        solver."""
        if self.route.stopLocations is None:
            self.stopLocations = []
        else:
            self.stopLocations = list(self.route.stopLocations)

    def next_stop(self):
        """Sets the next stop by popping it from the list."""
        try:
            return self.stopLocations.pop(0)
        except IndexError:
            return 1e16

    def distance_to_next_stop(self, distance):
        return self.nextStop - distance

    def rhs(self, t, states, sw):
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

        # bring in some useful info
        angle = self.route.current_angle(distance)
        speedLimit = self.route.current_speed_limit(distance)

        dstates = [0., 0., 0.]

        # speed
        dstates[0] = states[1]

        # acceleration
        Fd = self.bicyclist.aero_drag(speed, self.route.airDensity)
        Fi = self.bicyclist.incline_force(angle)
        Fr = self.bicyclist.rolling_friction(self.route.rollingFrictionCoefficient, speed, angle)

        if sw[0]:
            Fp = 0.0
            Fb = -self.bicyclist.max_brake_force(self.route.adhesionCoefficient, angle)
        elif speed > speedLimit:
            Fp = 0.0
            Fb = self.bicyclist.stop_brake_force(speed, speedLimit) - Fd - Fi - Fr
            #FbMax = -self.bicyclist.max_brake_force(self.route.adhesionCoefficient, angle)
            #if Fb < FbMax:
                #Fb = FbMax
        else:
            Fp = self.bicyclist.max_propulsion(speed, self.route.adhesionCoefficient, angle)
            Fb = 0.0

        forces = Fp + Fd + Fi + Fr + Fb
        dstates[1] = forces / self.bicyclist.mass

        # power
        dstates[2] = Fp * speed

        return dstates

    def event(self, t, y, sw):
        """Root is when the it is time to start braking."""
        distance = y[0]
        speed = y[1]
        distToStop = self.distance_to_next_stop(distance)
        if distToStop is None:
            ## TODO: this needs work. we basically do not want to return a root
            # after the last stop sign
            brake = 1.0
        else:
            angle = self.route.current_angle(distance)
            distToBrake = self.bicyclist.distance_to_stop(speed,
                    self.route.adhesionCoefficient, angle)
            brake = distToStop - distToBrake
        return [brake, speed]

    def solve(self, ):

        self.create_stop_list()

        # the switch:

        # RHS: right hand side function
        # ROOT: the event/root function
        # SW: the initial values of the switch, there should be one for each
        # root
        solver = sundials.CVodeSolver(RHS=self.rhs, ROOT=self.event, SW=[False,
            False], abstol = 1.0e-6, reltol = 1.0e-6)

        # The initial conditions such that the rider is riding just below the
        # starting speed limit.
        t0 = 0.0
        y0 = [0., self.route.speedLimit[0] - 0.1, 0.]
        y0 = [0., 1e-10, 0.]
        solver.init(t0, y0)

        dt = 1.0 # integration step size
        iterate = solver.iter(t0, dt)
        time = [t0]
        stateHistory = [y0]
        brakeLocations = []
        self.nextStop = self.next_stop()
        while True:
            try:
                t, y = next(iterate)
            except sundials.CVodeRootException, info:
                #print '-' * 20
                #print info.SW[0], info.SW[1]
                # if the braking point is found
                if info.SW[0] is True and info.SW[1] is False:
                    #print "The next stop is at {} meters.".format(self.nextStop)
                    #print "Found brake point at {} meters and speed is {} m/s.".format(info.y[0], info.y[1])
                    solver.SW[0] = True # brake for stop sign!
                    #solver.SW[1] = False
                    solver.init(info.t, [info.y[0], info.y[1], info.y[2]])
                    brakeLocations.append(info.y[0])
                    self.nextStop = self.next_stop()
                    time.append(info.t)
                    stateHistory.append([info.y[0], info.y[1], info.y[2]])
                elif info.SW[0] is False and info.SW[1] is True:
                    #print "The bicyclist has stopped at {} meters, v = {} m/s.".format(info.y[0], info.y[1])
                    solver.SW[0] = False # stop braking!
                    #solver.SW[1] = True
                    solver.init(info.t, [info.y[0], 1e-5, info.y[2]])
                    time.append(info.t)
                    stateHistory.append([info.y[0], 0.0, info.y[2]])
                #print '-' * 20
            except sundials.CVodeError:
                break
            else:
                time.append(t)
                stateHistory.append([y[0], y[1], y[2]])

            if y[0] > self.route.distance[-1]:
                break

        self.time = np.array(time)
        self.states = np.array(stateHistory)
        self.brakeLocations = np.array(brakeLocations)

    def stats(self):
        avgSpeed = self.states[:, 1].mean()
        totalDistance = self.states[-1, 0]
        totalEnergy = self.states[-1, 2]
        power = np.hstack((0.0, np.diff(self.states[:, 2]) / np.diff(self.time)))
        avgPower = power.mean()
        print 'Total distance traveled: {:1.0f} m'.format(totalDistance)
        print 'Trip time: {:1.0f} min'.format(self.time[-1] / 60.)
        print 'Average speed: {:1.2f} m/s'.format(avgSpeed)
        print 'Total energy spent: {:1.0f} J'.format(totalEnergy)
        print 'Average power: {:1.1f} W'.format(avgPower)

    def plot(self):

        fig, axes = plt.subplots(5, 1, squeeze=True, sharex=True)

        distance = self.states[:, 0]
        speed = self.states[:, 1]
        energy = self.states[:, 2]
        power = np.hstack((0.0, np.diff(energy) / np.diff(self.time)))

        elevation = np.interp(distance, self.route.distance, self.route.elevation)
        speedLimit = np.interp(distance, self.route.distance, self.route.speedLimit)


        axes[0].plot(self.time, elevation)
        axes[0].set_ylabel('Elevation [m]')

        axes[1].plot(self.time, distance)
        axes[1].set_ylabel('Distance [m]')

        axes[2].plot(self.time, self.states[:, 1], label='_nolabel')
        axes[2].plot(self.time, speedLimit, label='Speed Limit')
        axes[2].set_ylabel('Speed [m/s]')

        if self.route.stopLocations is not None:
            timeAtStop = np.interp(self.route.stopLocations, distance, self.time)
            elevationAtStop = np.interp(timeAtStop, self.time, elevation)
            speedAtStop = np.interp(timeAtStop, self.time, speed)

            timeAtBrake = np.interp(self.brakeLocations, distance, self.time)
            elevationAtBrake = np.interp(timeAtBrake, self.time, elevation)
            speedAtBrake = np.interp(timeAtBrake, self.time, speed)

            axes[0].plot(timeAtStop, elevationAtStop, 'or')
            axes[0].plot(timeAtBrake, elevationAtBrake, 'vy')

            axes[1].plot(timeAtStop, self.route.stopLocations[:len(timeAtStop)], 'or')
            axes[1].plot(timeAtBrake, self.route.stopLocations[:len(timeAtBrake)], 'vy')

            axes[2].plot(timeAtStop, speedAtStop, 'or')
            axes[2].plot(timeAtBrake, speedAtBrake, 'vy')

        axes[2].legend()

        axes[3].plot(self.time, self.states[:, 2])
        axes[3].set_ylabel('Energy [J]')

        axes[4].plot(self.time, power)
        axes[4].set_ylabel('Power [W]')
        axes[4].set_xlabel('Time [s]')

        return fig
