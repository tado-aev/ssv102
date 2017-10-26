#!/usr/bin/env python2

from osgeo import osr


class Converter:
    def __init__(self, src_epsg=4326, tgt_epsg=4978):
        self.src = osr.SpatialReference()
        self.src.ImportFromEPSG(src_epsg)
        self.tgt = osr.SpatialReference()
        self.tgt.ImportFromEPSG(tgt_epsg)
        self.coord_tf = osr.CoordinateTransformation(self.src, self.tgt)

    def convert(self, gga):
        x, y, z = self.coord_tf.TransformPoint(gga.lon, gga.lat, gga.height)
        return x, y, z
