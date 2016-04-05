#!/usr/bin/env python3
# -*- coding: <utf-8> -*-

"""Proxy between ardrone.ARDrone class and its CLI interface."""


import inspect
import ardrone
import sys


__drones__ = {}
__thismodule = sys.modules[__name__]

def exit(dronename=None):
    """Exit the prompt."""
    raise EOFError


def quit(dronename=None):
    """Quit the prompt."""
    raise EOFError


def help(commandname, dronename=None):
    """Print <commandname> description and signature."""
    command = getattr(__thismodule, commandname, None)

    if command is None:
        print('command <%s> is not supported' % commandname)
    else:
        sign = str(inspect.signature(command))[1:-1].replace(',', ' ')
        sign = ['<' + param + '>' for param in sign.split()]
        sign = ' '.join(filter(lambda param: '=' not in param, sign))

        print(' ')
        print('      \033[31mUsage\033[0m: %s %s' % (commandname, sign))
        print('\033[31mDescription\033[0m: %s' % inspect.getdoc(command))


def takeoff(dronename=None):
    """Take the drone off."""
    if dronename not in __drones__:
        __drones__[dronename] = ardrone.ARDrone(dronename)

    __drones__[dronename].takeoff()


def land(dronename=None):
    """Land the drone."""
    if dronename not in __drones__:
        __drones__[dronename] = ardrone.ARDrone(dronename)

    __drones__[dronename].land()


def reset(dronename=None):
    """Reset the drone."""
    if dronename not in __drones__:
        __drones__[dronename] = ardrone.ARDrone(dronename)

    __drones__[dronename].reset()


def goto(x, y, z, dronename=None):
    """Make drone go to 3D point."""
    if dronename not in __drones__:
        __drones__[dronename] = ardrone.ARDrone(dronename)

    result = __drones__[dronename].goto([x, y, z])
    if result != "success":
        print(result)
