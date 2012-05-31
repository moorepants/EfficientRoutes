import numpy as np

from efficientroutes import model

eps = 1e-15

def test_Bicyclist():
    mass = 84.0
    Cd = 1.16
    W_max = 120
    A = 0.56
    bicyclist = model.Bicyclist(mass=mass, dragCoefficient=Cd,
            frontalArea=A, maxPower=W_max, ridingStyle='casual')

    assert (bicyclist.mass - mass) < eps
    assert (bicyclist.dragCoefficient - Cd) < eps
    assert (bicyclist.frontalArea - A) < eps
    assert (bicyclist.maxPower - W_max) < eps
    assert bicyclist.ridingStyle == 'casual'

    g = 9.81
    assert (bicyclist.weight() - mass * g) < eps

    speed = 5.0
    rho = 1.2
    angle = 0.1
    drag = -np.sign(speed) * rho * speed**2 / 2.0 * Cd * A
    assert (bicyclist.aero_drag(rho, speed) - drag) < eps
    assert (bicyclist.normal_force(angle) - (mass * g) * np.cos(angle)) < eps
    assert (bicyclist.incline_force(angle) + (mass * g) * np.sin(angle)) < eps
    Cr = 0.008
    assert (bicyclist.rolling_friction(Cr, speed, angle) - (-np.sign(speed) *
        Cr * (mass * g) * np.cos(angle) * (1.0 - np.exp(-speed / 10.0)))) < eps
    Ca = 0.17
    assert (bicyclist.max_brake_force(Ca, angle) - Ca * mass * g *
        np.cos(angle)) < eps
    assert (bicyclist.max_propulsion(speed, Ca, angle) - min(W_max / speed, Ca
        * mass * g * np.cos(angle))) < eps

def test_Route():
    x = np.linspace(0.0, 100.0, num=100)
    z = np.sin(x)
    l = np.hstack((5.0 * np.ones_like(x[:50]), 10.0 * np.ones_like(x[50:])))
    s = np.array([45.0, 67.8, 92.0])
    route = model.Route(x, z, l, s)

    assert (route.current_speed_limit(30.0) - 5.0) < eps
    assert (route.current_speed_limit(60.0) - 10.0) < eps

    ns = route.next_stop(15.0)
    assert (ns - 45.0) < eps
    dts = route.distance_to_stop(15.0)
    assert (dts - 30.0) < eps

    ns = route.next_stop(50.0)
    assert (ns - 67.8) < eps
    dts = route.distance_to_stop(50.0)
    assert (dts - 17.8) < eps

    ns = route.next_stop(70.0)
    assert (ns - 92.0) < eps
    dts = route.distance_to_stop(70.0)
    assert (dts - 22.0) < eps

    ns = route.next_stop(95.0)
    assert ns is None
    dts = route.distance_to_stop(95.0)
    assert dts is None
