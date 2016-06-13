#!/usr/bin/env python2
# -*- coding: <utf-8> -*-

"""Interactive prompt for ARDrone class."""

from functools import partial
from types import ModuleType

import traceback
import os
import sys

from prompt_toolkit import prompt
from prompt_toolkit.contrib.completers import WordCompleter
from prompt_toolkit.history import FileHistory
from prompt_toolkit.auto_suggest import AutoSuggestFromHistory

from rospy import init_node
init_node('great_ardrones')

CURRENTPATH = os.path.dirname(os.path.abspath(__file__))
sys.path.append(CURRENTPATH)
import ardronecommands

from pygments.style import Style
from pygments.token import Token
from pygments.styles.default import DefaultStyle


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

    def func(word):
        cmd = getattr(ardronecommands, word)
        if word.startswith('__'):
            return False
        if not callable(cmd):
            return False
        if isinstance(cmd, ModuleType):
            return False

        return True

    commands = list(filter(func, dir(ardronecommands)))

    promptargs = {
        'vi_mode':      True,
        'true_color':   True,
        'auto_suggest': AutoSuggestFromHistory(),
        'completer':    WordCompleter(commands, ignore_case=True),
        'style':        document_style,
        'history':      FileHistory('.ardrone_cli_history')
    }

    drone_list = getattr(ardronecommands, '__drones__')

    while True:
        try:
            inputline = prompt(u'>>> ', **promptargs)

            for inputtext in inputline.split('&&'):
                inputtext = [str2num(s) for s in inputtext.split()]

                if len(inputtext) == 0:
                    continue

                dronename = None
                cmdname = inputtext[0]
                args = inputtext[1:]

                def prompt_error(*args, **kwargs):
                    """Print prompt incorrect command message."""
                    print('command <%s> is not supported' % cmdname)

                prompt_default = prompt_error

                if cmdname in drone_list and len(args) > 0:
                    dronename, cmdname, args = cmdname, args[0], args[1:]

                if len(args) == 0:
                    args = ()

                cmd = getattr(ardronecommands, cmdname, prompt_error)
                prompt_default = partial(cmd, dronename=dronename)
                getattr(ardronecommands, cmdname, prompt_default)(*args)

        except KeyboardInterrupt:
            continue

        except (TypeError, AttributeError):
            traceback.print_exc(file=sys.stdout)

            print('%s: wrong command signature' % cmdname)
            getattr(ardronecommands, 'help')(cmdname)

        except EOFError:
            break

    for drone in drone_list.values():
        drone.destroy()


if __name__ == '__main__':
    main()
