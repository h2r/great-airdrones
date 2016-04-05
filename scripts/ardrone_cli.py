#!/usr/bin/env python3
# -*- coding: <utf-8> -*-

"""Interactive prompt for ARDrone class."""


from prompt_toolkit import prompt
from prompt_toolkit.contrib.completers import WordCompleter
from prompt_toolkit.history import InMemoryHistory
from prompt_toolkit.auto_suggest import AutoSuggestFromHistory

from rospy import init_node
from functools import partial

from types import ModuleType

import traceback
import os
import sys

currentpath = os.path.dirname(os.path.abspath(__file__))
sys.path.append(currentpath)
import ardrone_cli_parser as parser

from pygments.style import Style
from pygments.token import Token
from pygments.styles.default import DefaultStyle


def is_public_method(module, word):
    """Check if the class member is public method."""
    if  word.startswith('__'):
        return False

    obj = getattr(module, word)
    if not callable(obj):
        return False

    if isinstance(obj, ModuleType):
        return False

    return True


def str2num(string):
    """Convert string to integer of floar number."""
    funcs = [float, str]

    for func in funcs:
        try:
            return func(string)
        except ValueError:
            pass


def main():
    """Ardrone CLI interface."""
    document_style = Style
    document_style.styles = {
        Token.Menu.Completions.Completion:         'bg:#555555 #ffa64d',
        Token.Menu.Completions.Completion.Current: 'bg:#888888 #ffffff',
    }

    document_style.styles.update(DefaultStyle.styles)

    init_node('great_ardrones')

    commands = filter(lambda word: is_public_method(parser, word), dir(parser))
    commands = [s.replace('_', ' ') for s in commands]

    promptargs = {
        'vi_mode':      True,
        'true_color':   True,
        'auto_suggest': AutoSuggestFromHistory(),
        'completer':    WordCompleter(commands, ignore_case=True),
        'style':        document_style,
        'history':      InMemoryHistory()
    }

    drone_list = getattr(parser, '__drones__')

    while True:
        try:
            inputtext = prompt('>>> ', **promptargs).split()
            inputtext = [str2num(s) for s in inputtext]

            if len(inputtext) == 0:
                continue

            def prompt_error(*args):
                """Print prompt incorrect command message."""
                print('command <%s> is not supported' % inputtext[0])

            prompt_default = prompt_error

            if inputtext[0] in drone_list and len(inputtext) > 1:
                cmd = getattr(parser, inputtext[1], prompt_error)
                prompt_default = partial(cmd, dronename=inputtext[0])

            command = getattr(parser, inputtext[0], prompt_default)

            if len(inputtext) == 0:
                command()
            else:
                command(*inputtext[1:], dronename='ardrone')

        except (TypeError, AttributeError):
            traceback.print_exc(file=sys.stdout)

            print('%s: wrong command signature' % inputtext[0])
            getattr(parser, 'help')(inputtext[0])

        except (EOFError, KeyboardInterrupt):
            break


if __name__ == '__main__':
    main()
