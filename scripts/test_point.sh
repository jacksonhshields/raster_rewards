#!/bin/bash

lon=148.00863
lat=-43.07754

rosservice call /get_point_value "point:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  status: {status: 0, service: 0}
  latitude: $lat
  longitude: $lon
  altitude: 0.0
  position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  position_covariance_type: 0"

