# Copyright (c) 2016
# Author: Victor Arribas <v.arribas.urjc@gmail.com>
# License: GPLv3 <http://www.gnu.org/licenses/gpl-3.0.html>

__author__ = 'varribas'

""" a very ugly method to simulate a 'print once'
it simply keeps an history of H, but grouped print once
and H=1 will be better """
# import hashlib
# hash_val = hashlib.md5(str).hexdigest()

__print_once_history = list()
HISTORY_LEN = 2


def set_history_len(h):
    global HISTORY_LEN
    HISTORY_LEN = h


def print_once(_str):
    global __print_once_history
    hash_val = hash(_str)
    if hash_val not in __print_once_history:
        __print_once_history.append(hash_val)
        n = max(0, len(__print_once_history)-HISTORY_LEN)
        __print_once_md5 = __print_once_history[n:]
        print _str
