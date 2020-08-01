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

Start the server:
```bash
roslaunch raster_rewards raster_reward.launch
```

Get the reward for a point (after sourcing the workspace):
```bash
lon=148.00863
lat=-43.07754
rosservice call /get_reward "latitude: $lat
longitude: $lon"
```