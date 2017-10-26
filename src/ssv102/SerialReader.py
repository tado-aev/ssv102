#!/usr/bin/env python2

import rospy

from serial import Serial, SerialException


class SerialReader:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        # Messages to read
        self.msgs = None
        try:
            self.com = Serial(port=port, baudrate=baud, timeout=5.0)
        except SerialException:
            rospy.logerr('Failed to open port {0}'.format(port))
            raise SystemExit

    def jascs(self, msg_rates):
        self.msgs = list(map(lambda s: s.lstrip('$'), msg_rates.keys()))
        fmt = '$JASC,{0},{1}\r\n'
        for k, v in msg_rates:
            self.com.write(fmt.format(k, v))

    def read_sentence(self):
        if self.msgs is None:
            return ''

        while not rospy.is_shutdown():
            line = self.com.readline()
            if line == '':
                continue
            msg_id = line.split(',')[0].lstrip('$')
            if msg_id in self.msgs:
                return line
