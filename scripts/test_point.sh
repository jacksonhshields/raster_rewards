#!/bin/bash

lon=148.00863
lat=-43.07754


rosservice call /get_reward "latitude: $lat
longitude: $lon"
