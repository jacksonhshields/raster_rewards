# Raster Rewards
This package is designed to provide rewards for a given point or trajectory.

## Requirements

- ROS  (needs python3)
- gdal (apt install libgdal-dev, or python3-gdal)
- osgeo (pip install GDAL - needs to match your gdal version)
- shapely (pip)
- pymap3d (pip)
- utm (pip)
- affine (pip)
- numpy (pip)


## Building

The services need to be built in order to run
```bash
cd /path/to/workspace
catkin_make
```

## Running

Source workspace:
```bash
source /path/to/workspace/devel/setup.bash
```

Start the server, giving the location to the reward tif:
```bash
roslaunch raster_rewards raster_reward.launch reward_raster_path:=/path/to/reward_raster.tif
```

Use the demo to get the reward for the path:
```bash
rosrun raster_rewards demo_client.py --csv /path/to/points.csv
```