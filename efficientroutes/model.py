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
            The switch boolean values, one for each event.

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

        if sw[0] is True:
            Fp = 0.0
            Fb = -self.bicyclist.max_brake_force(self.route.adhesionCoefficient, angle)
            power = 0.0
        elif sw[2] is True:
            Fp = 0.0
            Fb = self.bicyclist.stop_brake_force(speed, speedLimit) - Fd - Fi - Fr
            #FbMax = -self.bicyclist.max_brake_force(self.route.adhesionCoefficient, angle)
            #if Fb < FbMax:
                #Fb = FbMax

            if (Fd + Fr + Fi) < 0.0:
                power = np.min([self.bicyclist.max_propulsion(speed,
                    self.route.adhesionCoefficient, angle), -(Fd + Fr + Fi)]) * speed
            elif (Fd + Fr + Fi) > 0.0:
                power = 0.0

        else:
            Fp = self.bicyclist.max_propulsion(speed, self.route.adhesionCoefficient, angle)
            Fb = 0.0
            power = Fp * speed

        forces = Fp + Fd + Fi + Fr + Fb
        dstates[1] = forces / self.bicyclist.mass

        # power
        dstates[2] = power

        return dstates

    def event(self, t, y, sw):
        """Root is when the it is time to start braking.

        Parameters
        ----------
        t : float
            Current time.
        y : float
            Current states.
        sw : list
            Current switch values, one for each event.

        Returns
        -------
        brake : float
            This gives the distance to the next point at which braking should
            start. i.e. The distance required to stop from the current speed
            subtracted from distance to the next stopping point. When it goes
            from positive to negative, the brakes should be applied.
        stop : float
            This simply returns the value of the speed and is typically used to
            detect when the speed drops below zero.
        speeding : float
            This returns the difference in the speed limit and the current
            speed. It is used to detect when the bicyclist is speeding.

        """
        distance = y[0]
        speed = y[1]

        distToStop = self.distance_to_next_stop(distance)
        angle = self.route.current_angle(distance)
        distToBrake = self.bicyclist.distance_to_stop(speed,
                self.route.adhesionCoefficient, angle)
        brake = distToStop - distToBrake

        stop = speed

        speedLimit = self.route.current_speed_limit(distance)
        speeding = speedLimit - speed

        return brake, stop, speeding

    def solve(self, ):

        self.create_stop_list()
        self.nextStop = self.next_stop()

        solver = sundials.CVodeSolver(RHS=self.rhs, ROOT=self.event, SW=[False,
            False, False], abstol = 1.0e-6, reltol = 1.0e-6)

        # The initial conditions such that the rider is riding just below the
        # starting speed limit.
        t0 = 0.0
        y0 = [0., self.route.speedLimit[0] - 0.1, 0.]
        y0 = [0.0, 1e-10, 0.0]
        solver.init(t0, y0)

        dt = 1.0 # integration step size
        iterate = solver.iter(t0, dt)

        # collect the data in these lists
        time = [t0]
        stateHistory = [y0]
        brakeLocations = []

        while True:
            try:
                t, y = next(iterate)
            except sundials.CVodeRootException, info:
                #print '-' * 20
                #print "info", info.SW
                #print "solver", solver.SW
                # SW: [brake, stop, speed]
                # NOTE: The solver.SW can not be overwritten by a list, the
                # values must be set individually.
                if info.SW[0] is True:
                    #print "The next stop is at {} meters.".format(self.nextStop)
                    #print "Found brake point at {} meters and speed is {} m/s.".format(info.y[0], info.y[1])
                    solver.SW[0] = True
                    solver.SW[1] = False
                    solver.SW[2] = False
                    solver.init(info.t, [info.y[0], info.y[1], info.y[2]])
                    brakeLocations.append(info.y[0])
                    self.nextStop = self.next_stop()
                    time.append(info.t)
                    stateHistory.append([info.y[0], info.y[1], info.y[2]])
                elif info.SW[1] is True:
                    #print "The bicyclist has stopped at {} meters, v = {} m/s.".format(info.y[0], info.y[1])
                    solver.SW[0] = False
                    solver.SW[1] = False
                    solver.SW[2] = False
                    solver.init(info.t, [info.y[0], 1e-5, info.y[2]])
                    time.append(info.t)
                    stateHistory.append([info.y[0], 0.0, info.y[2]])
                elif info.SW[2] is True:
                    #speedLimit = self.route.current_speed_limit(info.y[0])
                    #print "Speed {} m/s exceeds the speed limit {} m/s".format(info.y[1],
                        #speedLimit)
                    solver.SW[0] = False
                    solver.SW[1] = False
                    solver.SW[2] = True
                    solver.init(info.t, [info.y[0], info.y[1], info.y[2]])
                    time.append(info.t)
                    stateHistory.append([info.y[0], info.y[1], info.y[2]])

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
        """Prints basic statistics about the latest solved run."""
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

        seconds2hours = 1. / 3600.

        distance = self.states[:, 0] / 1000.0 # km
        speed = self.states[:, 1] / 1000.0 / seconds2hours # km/h
        energy = self.states[:, 2] / 1000.0 # kilojoules
        power = np.hstack((0.0, np.diff(self.states[:, 2]) / np.diff(self.time))) # watts

        elevation = np.interp(distance, self.route.distance / 1000.0,
                self.route.elevation) # meters
        speedLimit = np.interp(distance, self.route.distance / 1000.0,
                self.route.speedLimit / 1000.0 / seconds2hours) # km/h

        meters2feet = 3.280839
        kilometers2miles = 0.621371192

        axes[0].plot(self.time / 60.0, elevation)
        axes[0].set_ylabel('Elevation [m]')
        axes[0].grid(True)
        english = axes[0].twinx()
        english.axis(np.hstack((axes[0].get_xlim(),
            np.array(axes[0].get_ylim()) *
            meters2feet)))
        english.set_ylabel('Elevation [ft]')

        axes[1].plot(self.time / 60.0, distance)
        axes[1].set_ylabel('Distance [km]')
        axes[1].grid(True)
        english = axes[1].twinx()
        english.axis(np.hstack((axes[1].get_xlim(),
            np.array(axes[1].get_ylim()) *
            kilometers2miles)))
        english.set_ylabel('Distance [miles]')

        axes[2].plot(self.time / 60.0, speed, label='_nolabel')
        axes[2].plot(self.time / 60.0, speedLimit, label='Speed Limit')
        axes[2].set_ylabel('Speed [km/h]')
        axes[2].grid(True)
        english = axes[2].twinx()
        english.axis(np.hstack((axes[2].get_xlim(),
            np.array(axes[2].get_ylim()) *
            kilometers2miles)))
        english.set_ylabel('Speed [mph]')

        if self.route.stopLocations is not None:
            stopLocations = self.route.stopLocations / 1000.0 # km
            brakeLocations = self.brakeLocations / 1000.0 # km

            timeAtStop = np.interp(stopLocations, distance, self.time)
            elevationAtStop = np.interp(timeAtStop, self.time, elevation)
            speedAtStop = np.interp(timeAtStop, self.time, speed)

            timeAtBrake = np.interp(brakeLocations, distance, self.time)
            elevationAtBrake = np.interp(timeAtBrake, self.time, elevation)
            speedAtBrake = np.interp(timeAtBrake, self.time, speed)

            axes[0].plot(timeAtStop / 60.0, elevationAtStop, 'or')
            axes[0].plot(timeAtBrake / 60.0, elevationAtBrake, 'vy')

            axes[1].plot(timeAtStop / 60.0, self.route.stopLocations[:len(timeAtStop)]
                    / 1000.0, 'or')
            axes[1].plot(timeAtBrake / 60.0,
                    self.route.stopLocations[:len(timeAtBrake)] / 1000.0, 'vy')

            axes[2].plot(timeAtStop / 60.0, speedAtStop, 'or')
            axes[2].plot(timeAtBrake / 60.0, speedAtBrake, 'vy')

        axes[2].legend()

        kilojoules2foodcal = 1. / 4.184

        axes[3].plot(self.time / 60.0, energy)
        axes[3].set_ylabel('Energy [kJ]')
        axes[3].grid(True)
        english = axes[3].twinx()
        english.axis(np.hstack((axes[3].get_xlim(),
            np.array(axes[3].get_ylim()) *
            kilojoules2foodcal)))
        english.set_ylabel('Energy [cal]')

        watt2hp = 0.00134102209

        axes[4].plot(self.time / 60.0, power)
        axes[4].set_ylabel('Power [W]')
        axes[4].set_xlabel('Time [min]')
        axes[4].grid(True)
        english = axes[4].twinx()
        english.axis(np.hstack((axes[4].get_xlim(),
            np.array(axes[4].get_ylim()) *
            watt2hp)))
        english.set_ylabel('Power [hp]')

        return fig
