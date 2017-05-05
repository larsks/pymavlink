#!/usr/bin/env python

import argparse
import itertools
import logging
import simplekml
import sys

from pymavlink import mavutil
from pymavlink import mavparser
from pymavlink.mavutil import mavlink

# This maps MAV_CMD_NAV_ values back to their names.
MAV_CMD_MAP = {getattr(mavlink, k): k
               for k in dir(mavlink)
               if k.startswith('MAV_CMD_NAV_')}

# For additional icon URLs:
# http://kml4earth.appspot.com/icons.html
MAV_CMD_SHAPES = {
    mavlink.MAV_CMD_NAV_WAYPOINT: 'http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png',
    mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH: 'http://maps.google.com/mapfiles/kml/paddle/red-square.png',
    mavlink.MAV_CMD_NAV_TAKEOFF: 'http://maps.google.com/mapfiles/kml/paddle/go.png',
    mavlink.MAV_CMD_NAV_LAND: 'http://maps.google.com/mapfiles/kml/paddle/orange-square.png',
    mavlink.MAV_CMD_NAV_LOITER_TIME: 'http://maps.google.com/mapfiles/kml/paddle/pause.png',
    mavlink.MAV_CMD_NAV_LOITER_TO_ALT: 'http://maps.google.com/mapfiles/kml/paddle/pause.png',
    mavlink.MAV_CMD_NAV_LOITER_TURNS: 'http://maps.google.com/mapfiles/kml/paddle/pause.png',
    mavlink.MAV_CMD_NAV_LOITER_UNLIM: 'http://maps.google.com/mapfiles/kml/paddle/pause.png',
}

DEFAULT_POSITION_SOURCE = 'GPS'

COLORS = [
    simplekml.Color.red,
    simplekml.Color.blue,
    simplekml.Color.violet,
    simplekml.Color.yellow,
    simplekml.Color.orange,
    simplekml.Color.burlywood,
    simplekml.Color.azure,
    simplekml.Color.lightblue,
    simplekml.Color.lawngreen,
    simplekml.Color.indianred,
    simplekml.Color.hotpink,
    simplekml.Color.green,
]

def coords(msg):
    return (getattr(msg, 'Lng', getattr(msg, 'Lon', None)),
            msg.Lat,
            msg.Alt)

class MavKML(mavparser.MAVParser):
    def __init__(self,
                 source=DEFAULT_POSITION_SOURCE,
                 mode_map=mavutil.mode_mapping_acm,
                 extrude_track=False,
                 extract_waypoints=False,
                 color_by_mode=False,
                 **kwargs):
        super(MavKML, self).__init__(**kwargs)

        self.source = source
        self.mode_map = mode_map
        self.extrude_track = extrude_track
        self.extract_waypoints = extract_waypoints
        self.color_by_mode = color_by_mode

        self.modes = {}
        self.mode_colors = itertools.cycle(COLORS)
        self.waypoints = set()
        self.kml = simplekml.Kml()

        self.register_handler(source, self.handle_position_msg)
        self.init_seq()

    def parse(self, source):
        self.cur_trk = None
        self.cur_mode = None
        self.cur_trk_color = None

        super(MavKML, self).parse(source)

    def init_seq(self):
        self.wptid_seq = itertools.count()
        self.trkid_seq = itertools.count()

    def new_track(self):
        trkid = next(self.trkid_seq)
        self.cur_trk = self.kml.newlinestring(
            name='TRK{:03d}'.format(trkid),
            altitudemode='absolute',
            extrude=self.extrude_track)

        if self.color_by_mode:
            if not self.cur_mode in self.modes:
                self.modes[self.cur_mode] = next(self.mode_colors)
            self.cur_trk.linestyle.color = self.modes[self.cur_mode]

        if self.cur_trk_color:
            self.cur_trk.linestyle.color = self.cur_trk_color

        md = simplekml.ExtendedData()
        md.newdata('mode_id', self.cur_mode)
        md.newdata('mode_name', self.mode_map.get(self.cur_mode, 'UNKNOWN'))
        self.cur_trk.extendeddata = md

    def on_MODE(self, msg):
        '''Close the current track on mode switches'''

        if msg.Mode != self.cur_mode:
            self.cur_trk = None
            self.cur_mode = msg.Mode

    def handle_position_msg(self, msg):
        '''Add position information to the current track'''
        wp = coords(msg)
        if self.cur_trk is None:
            self.new_track()

        self.cur_trk.coords.addcoordinates([wp])

    def on_CMD(self, msg):
        '''Extract coordinates from MAV_CMD_NAV_* messages and add them as
        waypoints to generated KML.'''

        # Abort if we are not trying to extract waypoints.
        if not self.extract_waypoints:
            return

        # We only care about MAV_CMD_NAV_* commands.
        if msg.CId not in MAV_CMD_MAP:
            return

        wp = coords(msg)

        # Skip this waypoint if we have already logged one at the same
        # location.
        if wp in self.waypoints:
            return

        self.waypoints.add(wp)
        wptid = next(self.wptid_seq)

        md = simplekml.ExtendedData()
        md.newdata('command_id', msg.CId)
        md.newdata('command_name', MAV_CMD_MAP.get(msg.CId, 'UNKNOWN'))
        pnt = self.kml.newpoint(name='WP{:03d}'.format(wptid),
                                coords=[wp],
                                altitudemode='absolute',
                                extendeddata=md)

        if msg.CId in MAV_CMD_SHAPES:
            style = simplekml.Style()
            style.iconstyle.icon.href = MAV_CMD_SHAPES[msg.CId]
            pnt.style = style

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--source', '-s',
                   default=DEFAULT_POSITION_SOURCE,
                   help=(
                       'Message type for position information '
                       '(defaults to {})'.format(DEFAULT_POSITION_SOURCE)))
    p.add_argument('--output', '-o',
                   help='Output KML to named file (defaults to stdout)')

    g = p.add_argument_group('Filtering options')
    g.add_argument('--condition', '-c',
                   help='Filter mavlink messages on CONDITION')
    g.add_argument('--waypoints', '-w',
                   action='store_true',
                   help='Extract waypoints from log')

    g = p.add_argument_group('Display options')
    g.add_argument('--color-by-mode', '-k',
                   action='store_true',
                   help='Color tracks by mode')
    g.add_argument('--extrude', '-e',
                   action='store_true',
                   help='Extend tracks to ground')

    g = p.add_argument_group('Logging options')
    g.add_argument('--quiet', '-q',
                   action='store_const',
                   const='WARNING',
                   dest='loglevel')
    g.add_argument('--debug', '-d',
                   action='store_const',
                   const='DEBUG',
                   dest='loglevel')

    p.set_defaults(loglevel='INFO')

    p.add_argument('logfile', nargs='*')

    return p.parse_args()

def main():
    args = parse_args()
    logging.basicConfig(level=args.loglevel)
    p = MavKML(condition=args.condition,
               extrude_track=args.extrude,
               extract_waypoints=args.waypoints,
               color_by_mode=args.color_by_mode)

    for logfile in args.logfile:
        p.parse(logfile)

    with open(args.output, 'w') if args.output else sys.stdout as fd:
        fd.write(p.kml.kml())

if __name__ == '__main__':
    main()
