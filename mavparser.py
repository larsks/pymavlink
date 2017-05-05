#!/usr/bin/env python

import sys
import logging
import json
import datetime
from pymavlink import mavutil

LOG = logging.getLogger(__name__)


class MAVParser (object):
    def __init__(self, condition=None):
        self.condition = condition
        self.handlers = {}
        for attr in dir(self):
            if attr.startswith('on_'):
                self.register_handler(attr[3:], getattr(self, attr))

    def register_handler(self, msgtype, handler):
        '''Register a function to handle messages of the given type'''

        LOG.debug('registering handler for %s messages', msgtype)
        self.handlers[msgtype] = handler

    def get_next_message(self):
        m = self.mlog.recv_match(type=list(self.handlers),
                                 condition=self.condition,
                                 blocking=True)
        return m

    def parse(self, src):
        '''Read messages from a mavlink stream.'''

        LOG.info('processing messages from %s', src)

        self.mlog = mavutil.mavlink_connection(src)
        msgcount = 0

        while True:
            m = self.get_next_message()
            if m is None:
                break

            msgcount += 1
            LOG.debug('processing %s message: %s', m.get_type(), m)
            handler = self.handlers[m.get_type()]
            handler(m)

        LOG.info('finished processing %s (%d messages)', src, msgcount)
