#!/usr/bin/env python

import sys
import logging
import json
import datetime
from pymavlink import mavutil

LOG = logging.getLogger(__name__)


class MAVParser (object):

    condition = None

    def __init__(self):
        self.msgtypes = set()
        for attr in dir(self):
            if attr.startswith('on_'):
                self.msgtypes.add(attr[3:])

        LOG.info('handling message types: %s',
                ','.join(self.msgtypes))

    def parse(self, src):
        mlog = mavutil.mavlink_connection(src)

        while True:
            m = mlog.recv_match(type=list(self.msgtypes), condition=self.condition)
            if m is None:
                break

            handler = getattr(self, 'on_{}'.format(m.get_type()))
            handler(m)

class MavToGis (MAVParser):

    def __init__(self):
        super(MavToGis, self).__init__()
        self.curpos = {}
        self.curtypes = set()
        self.fix = False

    def init_output(self):
        self.outfd.write('{"type": "FeatureCollection", '
                         '"features": [')

    def finish_output(self):
        self.outfd.write(']}')

    def write_curpos(self):
        if not self.curpos:
            return

        if self.curtypes != self.msgtypes:
            missing = self.msgtypes - self.curtypes
            LOG.warning('incomplete data point; missing: %s',
                        ','.join(missing))
            return

        if not 'vfr_hud_airspeed' in self.curpos:
            sys.stdout.flush()
            print self.curtypes
            print self.curpos
            sys.exit()

        if not self.fix:
            LOG.debug('ignoring data with no gps fix')
            return

        point = {'type': 'Feature',
                 'geometry': {
                     'type': 'Point',
                     'coordinates': [
                         self.curpos['gps_lon'],
                         self.curpos['gps_lat']]},
                 'properties': self.curpos,
                 }

        if not self.firstpoint:
            self.outfd.write('\n,\n')

        self.firstpoint = False
        self.outfd.write(json.dumps(point))

    def on_GPS_RAW_INT(self, msg):
        self.write_curpos()

        self.fix = msg.fix_type >= 2

        self.curpos = dict(
            gps_lat=msg.lat/1.0e7,
            gps_lon=msg.lon/1.0e7,
            gps_alt=msg.alt/1.0e3,
            gps_velocity=msg.vel/100.0,
            gps_heading=msg.cog/100.0,
            timestamp=datetime.datetime.fromtimestamp(msg._timestamp).isoformat()
            )
        self.curtypes = {msg.get_type()}

    def on_VFR_HUD(self, msg):
        self.curtypes.add(msg.get_type())
        for field in msg.fieldnames:
            self.curpos['{}_{}'.format(
                msg.get_type().lower(), field)] = getattr(msg, field)

    def on_VIBRATION(self, msg):
        self.curtypes.add(msg.get_type())
        for field in msg.fieldnames:
            self.curpos['{}_{}'.format(
                msg.get_type().lower(), field)] = getattr(msg, field)

    def on_RADIO_STATUS(self, msg):
        self.curtypes.add(msg.get_type())
        for field in msg.fieldnames:
            self.curpos['{}_{}'.format(
                msg.get_type().lower(), field)] = getattr(msg, field)

    def parse(self, src, outfd):
        self.outfd = outfd
        self.firstpoint = True
        self.init_output()

        try:
            super(MavToGis, self).parse(src)
        finally:
            self.finish_output()

logging.basicConfig(level='INFO')
parser = MavToGis()
parser.parse(sys.argv[1], sys.stdout)
