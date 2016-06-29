#!/usr/bin/env python3

# -*- coding: <utf-8> -*-

"""Proxy between ardrone.ARDrone class and its CLI interface."""


import inspect
import ardrone
import sys
import time
import os
import re
import rosgraph


__topics__ = [topic for topic in rosgraph.Master('/rostopic').getSystemState()]
__topics__ = [topic[0] for item in __topics__ for topic in item]
__topics__ = filter(lambda u: re.match('/.*/navdata', u) is not None, __topics__)
__dronenames__ = map(lambda u: os.path.dirname(u)[1:], __topics__)

__drones__ = {name: ardrone.ARDrone(name) for name in __dronenames__}

if len(__drones__) == 0:
    print("I don't see any drones created")
    quit()

__firstdrone__ = list(__drones__.values())[0]
__thismodule = sys.modules[__name__]


def exit():
    """Exit the prompt."""
    raise EOFError


def quit():
    """Quit the prompt."""
    raise EOFError


def clear():
    """Clear the screen."""
    os.system('clear')


def help(commandname):
    """Print <commandname> description and signature."""
    command = getattr(__thismodule, commandname, None)

    if command is None:
        print('command <%s> is not supported' % commandname)
    else:
        sign = str(inspect.signature(command))[1:-1].replace(',', ' ')
        sign = re.sub('\S*=<.*>','', sign)
        sign = ['<' + param + '>' for param in sign.split()]
        sign = ' '.join(filter(lambda param: '=' not in param, sign))

        print(' ')
        print('      \033[31mUsage\033[0m: %s %s' % (commandname, sign))
        print('\033[31mDescription\033[0m: %s' % inspect.getdoc(command))


def takeoff(drone=__firstdrone__):
    """Take the drone off."""
    drone.takeoff()


def land(drone=__firstdrone__):
    """Land the drone."""
    drone.land()

def freeze(drone=__firstdrone__):
    """Freeze drone"""
    drone.freeze()


def reset(drone=__firstdrone__):
    """Reset the drone."""
    drone.reset()

def settwist(x, y, z, wz, drone=__firstdrone__):
    """Set the drone twist."""
    drone.settwist([x, y, z], [0, 0, wz])


def getposition(drone=__firstdrone__):
    """Get drone's vrpn position."""
    droneposition = drone.getposition()
    if droneposition is None:
        print("I can't get any vrpn data related to the drone")
        return False

    print('x: %.4f\ny: %.4f\nz: %.4f\n' % tuple(droneposition))

    return True


def goto(x, y, z, drone=__firstdrone__):
    """Go to the absolute position."""
    result = drone.goto([x, y, z], False)

    if result != "success":
        print(result)


def rgoto(x, y, z, drone=__firstdrone__):
    """Go to the relative position."""
    result = drone.move([x, y, z], False)

    if result != "success":
        print(result)


def tgoto(x, y, z, drone=__firstdrone__):
    """Twist first and go to the absolute position."""
    result = drone.goto([x, y, z], True)

    if result != "success":
        print(result)


def trgoto(x, y, z, drone=__firstdrone__):
    """Twist first and go to the relative position."""
    result = drone.move([x, y, z], True)

    if result != "success":
        print(result)


def hover(drone=__firstdrone__):
    """Hover in place manually, not using built-in-hover"""
    drone.hover()
