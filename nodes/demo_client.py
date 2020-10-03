#!/usr/bin/env python

import time
import argparse
import sys
import numpy as np
import rospy
import pandas as pd
from sensor_msgs.msg import NavSatFix
from raster_rewards.srv import RasterPointReward, RasterPointRewardResponse, RasterPathReward, RasterPathRewardResponse

def get_args():
    parser = argparse.ArgumentParser(description="Sends a csv of points to a node")
    parser.add_argument('--csv', type=str, help="Path to the CSV file of points. Needs to have columns Latitude and Longitude", required=True)
    parser.add_argument('--sample-dist', type=float, default=2.0, help="The spatial distance at which to sample the reward map")
    args = parser.parse_args()
    return args


if __name__ == "__main__":
    rospy.init_node('demo_client', anonymous=True)
    args = get_args()
    df = pd.read_csv(args.csv)
    coords = []
    for index, row in df.iterrows():
        msg = NavSatFix()
        msg.latitude = row['Latitude']
        msg.longitude = row['Longitude']
        coords.append(msg)

    rospy.wait_for_service('get_path_value')
    try:
        service_proxy = rospy.ServiceProxy('get_path_value', RasterPathReward())
        res = service_proxy(coords, args.sample_dist)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    print("Reward of %f" %res.reward)
    rospy.spin()
