# ssV-102 Driver

This is a wrapper driver for the Hemisphere ssV-102 GPS. This device has two
GPS antennas, which allows it to calculate the heading. The driver utilizes
the `nmea_navsat_driver` for parsing GGA sentences.

## Dependencies

- [python-gdal](https://pypi.python.org/pypi/GDAL)
- [python-serial](https://pythonhosted.org/pyserial/)

## Recommended

- [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver)

## Parameters

- `port`: the port which the ssV-102 is connected to
- `baudrate`: baud rate of the device
- `jascs`: dictionary of the message type and the rate
- `publish_tf`: whether to publish tf transforms
- `gps_origin_frame`: only for tf. frame id for the GPS origin
- `gps_antenna_frame`: frame id for the antenna
- `gps_required_quality`: required data quality to publish tf
- `gps_src_epsg`: EPSG of the GGA sentence (should be fine with 4326)
- `gps_tgt_epsg`: target EPSG for projecting llh to xyz

## Subscribed topics

None

## Published topics

- `nmea_sentence`: used by `nmea_navsat_driver`

Published by `nmea_navsat_driver`:

- `fix`
- `vel`
- `time_reference`
- `heading`: https://github.com/ros-drivers/nmea_navsat_driver/pull/25

See the `nmea_navsat_driver` wiki page for more details on the topics.

## License

MIT License

## Author

Naoki Mizuno
