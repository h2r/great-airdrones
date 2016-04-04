#!/usr/bin/env python3
# -*- coding: <utf-8> -*-

"""CLI Parser"""
from ardrone import ARDrone

ardrone = ARDrone("ardrone")


def takeoff(*args):
    ardrone.takeoff()


def land(*args):
    ardrone.land()


def reset(*args):
    ardrone.reset()


def goto(*args):
    print("going")
    ardrone.goto(map(int, [args[0], args[1], args[2]]))
