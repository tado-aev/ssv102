#!/usr/bin/env python2

import tf.transformations

import math


class Parser(object):
    @staticmethod
    def parse(sentence):
        if sentence is None or sentence == '':
            return None, None

        rest, checksum = sentence.split('*')
        msg_id = rest.split(',')[0].lstrip('$')
        vals = rest.split(',')[1:]

        if any(map(lambda t: t == '', vals)):
            return None, None

        if msg_id == 'GPGGA':
            return msg_id, Parser.parse_gga(vals, checksum)
        elif msg_id == 'GPHDT':
            return msg_id, Parser.parse_hdt(vals, checksum)
        else:
            return None, None

    @staticmethod
    def parse_gga(vals, checksum):
        if len(vals) != 14:
            return None
        return GGA(vals)

    @staticmethod
    def parse_hdt(vals, checksum):
        if len(vals) != 2:
            return None
        return HDT(vals)


class GGA:
    def __init__(self, vals):
        self.utc, self.lat, self.lat_dir, \
            self.lon, self.lon_dir, self.quality, self.num_sat, \
            self.hdop, self.orthometric_height, \
            self.orthometric_height_unit, self. geoid_sep, \
            self.geoid_sep_unit, self.age, self.ref_station_id = vals

        # 3820.9247885 -> 38 deg 20.924788' -> 38 + 20.924788 / 60 -> 38.348746475
        self.lat = float(self.lat)
        self.lat = math.floor(self.lat / 100) + (self.lat % 100) / 60
        self.lon = float(self.lon)
        self.lon = math.floor(self.lon / 100) + (self.lon % 100) / 60
        self.orthometric_height = float(self.orthometric_height)
        self.geoid_sep = float(self.geoid_sep)

        self.height = self.orthometric_height + self.geoid_sep


class HDT:
    def __init__(self, vals):
        self.heading, self.true_heading = vals
        # Heading: CW is positive
        rad = math.radians(-float(self.heading))
        self.orientation = tf.transformations.quaternion_from_euler(0, 0, rad)
