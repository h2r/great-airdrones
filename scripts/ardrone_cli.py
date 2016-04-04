#!/usr/bin/env python3
# -*- coding: <utf-8> -*-

"""Interactive prompt for ARDrone class."""


import sys
from prompt_toolkit import prompt
from prompt_toolkit.contrib.completers import WordCompleter
from prompt_toolkit.completion import Completer
from prompt_toolkit.completion import Completion
from prompt_toolkit.history import InMemoryHistory
from prompt_toolkit.auto_suggest import AutoSuggestFromHistory

from pygments.style import Style
from pygments.token import Token
from pygments.styles.default import DefaultStyle

from rospy import init_node

import ardrone_cli_parser


class DocumentStyle(Style):
    """Colors for autocompletion."""

    styles = {
        Token.Menu.Completions.Completion:         'bg:#008888 #ffffff',
        Token.Menu.Completions.ProgressButton:     'bg:#003333',
        Token.Menu.Completions.ProgressBar:        'bg:#00aaaa',
        Token.Menu.Completions.Completion.Current: 'bg:#00aaaa #000000',
    }

    styles.update(DefaultStyle.styles)



def main():
    """Ardrone CLI interface."""
    init_node('great_ardrones')

    commands = filter(callable, dir(ardrone_cli_parser))
    commands = [s for s in commands if not s.startswith('__')]

    promptargs = {
        'vi_mode':      True,
        'auto_suggest': AutoSuggestFromHistory(),
        'completer':    WordCompleter(commands, ignore_case=True),
        'style':        DocumentStyle,
        'history':      InMemoryHistory()
    }

    def error_cmd(*args):
        print("%s: not supported" % args[0])

    while True:
        try:
            inputtext = prompt(u'>>> ', **promptargs).split()
            if len(inputtext) == 0:
                continue

            if inputtext[0] == "exit":
                sys.exit(0)
            if len(inputtext) > 1:
                getattr(ardrone_cli_parser, inputtext[0], error_cmd)(*inputtext[1:])
            else:
                getattr(ardrone_cli_parser, inputtext[0], error_cmd)([])
        except EOFError:
            break


if __name__ == '__main__':
    main()
