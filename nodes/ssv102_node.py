#!/usr/bin/env python2

import rospy
import tf
from nmea_msgs.msg import Sentence

from ssv102 import Parser, SerialReader, Converter


rospy.init_node('ssv102_node')

port = rospy.get_param('port')
baud = rospy.get_param('baud')
msg_rates = rospy.get_param('msg_rates', {
    'GPGGA': 10,
    'GPHDT': 10,
})
publish_tf = rospy.get_param('publish_tf', True)
gps_origin_frame = rospy.get_param('gps_origin_frame', 'gps_origin')
gps_antenna_frame = rospy.get_param('gps_antenna_frame', 'ssv102')
gps_required_quality = rospy.get_param('gps_required_quality', ['1', '2'])
gps_src_epsg = rospy.get_param('gps_src_epsg', 4326)
gps_tgt_epsg = rospy.get_param('gps_tgt_epsg', 4987)

serial_reader = SerialReader(port, baud)
serial_reader.jascs(msg_rates)
converter = Converter(gps_src_epsg, gps_tgt_epsg)

tf_b = tf.TransformBroadcaster()
nmea_pub = rospy.Publisher('nmea_sentence', Sentence, queue_size=10)

msg = Sentence()
seq = 0
# Contains the latest GPGGA and GPHDT
latest = {}
while not rospy.is_shutdown():
    t = rospy.Time.now()
    msg.header.stamp = t
    msg.header.frame_id = gps_antenna_frame
    msg.header.seq = seq
    seq += 1
    msg.sentence = serial_reader.read_sentence()
    nmea_pub.publish(msg)

    if not publish_tf:
        continue

    # Publish tf
    msg_id, obj = Parser.parse(msg.sentence)
    latest[msg_id] = obj

    if 'GPGGA' in latest.keys() and latest['GPGGA'] is not None \
            and 'GPHDT' in latest.keys() and latest['GPHDT'] is not None:
        # Data quality not enough
        if latest['GPGGA'].quality not in gps_required_quality:
            continue

        tf_b.sendTransform(converter.convert(latest['GPGGA']),
                           latest['GPHDT'].orientation,
                           t,
                           gps_antenna_frame,
                           gps_origin_frame)
