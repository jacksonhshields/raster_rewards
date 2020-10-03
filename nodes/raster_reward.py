#!/usr/bin/env python3


import numpy as np
import rospy
import shapely
import shapely.geometry
import utm
import warnings
from raster_rewards.srv import RasterPointReward, RasterPointRewardResponse, RasterPathReward, RasterPathRewardResponse, RasterExtent, RasterExtentResponse
from osgeo import osr
from affine import Affine
import pymap3d
from sensor_msgs.msg import NavSatFix

try:
    from osgeo import gdal
except ImportError:
    import gdal

def retrieve_pixel_coords(geo_coord,geot_params):
    """
    Gets the pixel coordinates for a given coordinate.
    Args:
        geo_coord: (np.ndarray) The coordianates
        geot_params: (list) The geotransform parameters. (from gdal.GetGeotransform())

    Returns:
        np.ndarray: The pixel coordinates of the raster corresponding to the given coordinate.

    """
    x, y = geo_coord[0], geo_coord[1]
    forward_transform =  Affine.from_gdal(*geot_params)
    reverse_transform = ~forward_transform
    px, py = reverse_transform * (x, y)
    px = np.around(px).astype(int)
    py = np.around(py).astype(int)
    pixel_coord = px, py
    return pixel_coord

class RewardMap:
    """
    Reward Map function is used to get rewards from a raster file.
    """
    def __init__(self, raster_path):
        """
        Loads the raster file.

        Args:
            raster_path: (str) The path to the raster file.
        """
        # Load the raster
        self.reward_raster = gdal.Open(raster_path)
        self.geotransform = self.reward_raster.GetGeoTransform()
        self.projection = self.reward_raster.GetProjection()
        # Whether to use UTM. If not, assumes lat,lon projection.
        self.use_utm = True if 'UTM' in self.projection else False
        # Sets the origin of the map. This is useful if you are using local coordinates.
        self.origin = [self.geotransform[0], # minimum x value
                       self.geotransform[3] + self.reward_raster.RasterYSize * self.geotransform[5]]  # max y value + num_y_pixels * pixel_height

        # Sets the bounds of the map
        self.bounds_geo = shapely.geometry.box(self.origin[0],  # Minimum x value
                                               self.origin[1],  # Minimum y vale
                                               self.origin[0] + self.reward_raster.RasterXSize*self.geotransform[1],  # Max x value = origin + num_x_pixels * pixel_width
                                               self.geotransform[3])  # Max y value
        # Sets the local bounds
        if self.use_utm:
            self.bounds_local = shapely.geometry.box(0.0,  # Bounds local starts at 0,0
                                                     0.0,  # Bounds local starts at 0,0
                                                     self.bounds_geo.bounds[2] - self.origin[0],
                                                     self.bounds_geo.bounds[3] - self.origin[1])
        else:
            dl = pymap3d.geodetic2enu(self.bounds_geo.bounds[0], self.bounds_geo.bounds[1], 0.0,
                                      self.origin[0], self.origin[1], 0.0) # down left
            ul = pymap3d.geodetic2enu(self.bounds_geo.bounds[0], self.bounds_geo.bounds[3], 0.0,
                                      self.origin[0], self.origin[1], 0.0) # up left
            dr = pymap3d.geodetic2enu(self.bounds_geo.bounds[2], self.bounds_geo.bounds[1], 0.0,
                                      self.origin[0], self.origin[1], 0.0) # down right
            ur = pymap3d.geodetic2enu(self.bounds_geo.bounds[2], self.bounds_geo.bounds[3], 0.0,
                                      self.origin[0], self.origin[1], 0.0) # up right
            self.bounds_local = shapely.geometry.box(min(dl[0], ul[0]),
                                                     min(dl[1], dr[1]),
                                                     max(dr[0], ur[0]),
                                                     max(ul[1], ur[1]))

    def geo_to_local(self, point):
        """
        Converts a geo point to a local one. Can either be utm or llh.
        Args:
            point: (np.ndarray) The point to convert. For utm: (x,y), for llh: (lat,lon)
        Returns:
            np.ndarray: (x,y) point

        """
        if self.use_utm:
            local_point = np.array([point[0] - self.origin[0], point[1] - self.origin[1]])
        else:
            # point[0] = latitude, point[1] = longitude
            enu = pymap3d.geodetic2enu(point[0], point[1], 0.0, self.origin[0], self.origin[1], 0.0)
            local_point = np.array(enu[:2])
        return local_point

    def local_to_geo(self, point):
        """
        Converts a local point to a geo one.
        Args:
            point: (np.ndarray) The point to convert (x,y)
        Returns:
            np.ndarray: The geo_point, which can either be utm or llh.

        """
        if self.use_utm:
            geo_point = np.array([point[0] + self.origin[0], point[1] + self.origin[1]])
        else:
            llh = pymap3d.enu2geodetic(point[0], point[1], 0.0, self.origin[0], self.origin[1], 0.0) # lat, lon, height
            geo_point = np.array(llh[:2])
        return geo_point

    def geo_to_llh(self, point):
        """
        Converts a geo point, which could be either utm or llh, to llh.
        Args:
            point: (np.ndarray) The geo point, either utm or llh

        Returns:
            np.ndarray: Lat,Lon

        """
        if self.use_utm:
            srs = osr.SpatialReference(wkt=self.projection)
            projcs = srs.GetAttrValue('projcs')
            if 'zone' in projcs:
                zonestr = projcs.split(' ')[-1]
                zone_num = int(zonestr[:2])
                zone_hem = zonestr[-1]
                if zone_hem == "N":
                    northern = True
                elif zone_hem == "S":
                    northern = False
                else:
                    raise ValueError("Zone hemisphere has to be either north or south")
            else:
                raise ValueError("Projection doesn't contain zone")
            latlon = list(utm.to_latlon(point[0], point[1], zone_num, None, northern=northern))
            ll = np.array(latlon)
        else:
            ll = np.array(point[:2])
        return ll

    def llh_to_geo(self, point):
        """
        Converts lat lon coordinate to geo. This can either be lat,lon (passthrough) or UTM.
        Args:
            point: (np.ndarray) The point to convert

        Returns:
            np.ndarray: The point in geo
        """
        if self.use_utm:
            xy = list(utm.from_latlon(point[0], point[1]))[:2]
            return np.array(xy)
        else:
            return point

    def get_bounds(self, geo):
        """
        Gets the bounds for the given array.

        Args:
            geo: (bool) Whether to use geo coordinates.

        Returns:
            tuple: Bounds in the form [minx,miny,maxx,maxy]
        """
        if geo:
            return self.bounds_geo.bounds
        else:
            return self.bounds_local.bounds

    def get_point_reward(self, point, geo):
        """
        Gets the point reward for a given coordinate
        Args:
            point: (np.ndarray) The point to find the coordinates for. If geo coordinates, it must match the projection of the raster.
            geo: (bool) Whether this point is in geo coordinates.

        Returns:

        """
        if geo:
            geo_point = point
        else:
            geo_point = self.local_to_geo(point)

        if self.use_utm:
            geo_xy = [geo_point[0], geo_point[1]]
        else: # geo_point for llh is lat,lon,alt, switch to lon,alt to conform to x,y
            geo_xy = [geo_point[1], geo_point[0]]
        px,py = retrieve_pixel_coords(geo_xy, geot_params=self.geotransform)
        reward = self.reward_raster.GetRasterBand(1).ReadAsArray(int(px), int(py), 1, 1)
        return reward

    def get_path_reward(self, path, sample_dist, geo):
        """
        Gets the reward for a path.
        Args:
            path: (list[np.ndarray]) The path as a list of points.
            sample_dist: (float) The interval along the path at which to sample. Pass in None to not subsample the path.
            geo: (bool) Whether to use geo coordinates

        Returns:
            float: The accumulated reward for the given path
        """

        if sample_dist is None:  # Only use this if path is presampled.
            path_sampled = path
        else:
            # Convert the geo points to local. Necessary for when geo is lat,lon
            local_points = []
            for point in path:
                if geo:
                    local_points.append(self.geo_to_local(point))
                else:
                    local_points.append(point)
            # Sample the path at a fixed spatial interval
            path_sampled_local = []
            leftover = 0.0
            for n in range(len(local_points) -1):
                pa = local_points[n]
                pb = local_points[n+1]
                ls = shapely.geometry.LineString([pa, pb])
                travelled = leftover
                while travelled < ls.length:
                    point = ls.interpolate(travelled)
                    travelled += sample_dist
                    path_sampled_local.append(np.array(point))
                leftover = ls.length - travelled
            # Convert the local points back to geo
            path_sampled = []
            for point in path_sampled_local:
                path_sampled.append(self.local_to_geo(point))
        # Get the rewards for the points
        rewards = []
        for point in path_sampled:
            rewards.append(self.get_point_reward(point,geo=geo))
        return np.sum(np.array(rewards))


class RewardNode:
    """
    A ROS node interface to the reward map.
    """
    def __init__(self):
        self.reward_raster_path = rospy.get_param("~reward_raster_path", "")
        print("Raster reward path", self.reward_raster_path)
        self.reward_map = RewardMap(self.reward_raster_path)
        
        self.reward_point_service = rospy.Service('get_point_value', RasterPointReward, self.rewardPointServiceCallback)
        self.reward_path_service = rospy.Service('get_path_value', RasterPathReward, self.rewardPathServiceCallback)
        self.extent_path_service = rospy.Service('get_extent', RasterExtent, self.extentServiceCallback)

    
    def rewardPointServiceCallback(self, req):
        """
        Gets the point reward for the requested point

        Args:
            req: (RasterPointReward) The request containing coordinates.

        Returns:
            RasterPointRewardResponse: The response containing the reward.

        """
        point = np.array([req.point.latitude, req.point.longitude])
        
        res = RasterPointRewardResponse()
        geopoint = self.reward_map.llh_to_geo(point=point)
        res.reward = self.reward_map.get_point_reward(point=geopoint, geo=True)
        
        return res

    def rewardPathServiceCallback(self, req):
        """
        Gets the reward for a given path
        Args:
            req: (RasterPathReward) The request containing the path of coordinates to get a reward for.

        Returns:
            RasterPathRewardResponse: The response containing the reward.

        """
        points = []
        for fix in req.path:
            point = np.array([fix.latitude, fix.longitude])

            geopoint = self.reward_map.llh_to_geo(point=point)
            points.append(geopoint)
        res = RasterPathRewardResponse()
        res.reward = self.reward_map.get_path_reward(points, sample_dist=req.sample_distance, geo=True)
        return res

    def extentServiceCallback(self, req):
        """
        Gets the extent of the raster (used for bounds)
        Args:
            req: (RasterExtent) blank

        Returns:
            RasterExtentResponse: The response containing the xmin,ymin,xmax,ymax

        """
        bounds = self.reward_map.get_bounds(geo=True)
        res = RasterExtentResponse()
        res.xmin = bounds[0]
        res.ymin = bounds[1]
        res.xmax = bounds[2]
        res.ymax = bounds[3]
        return res


if __name__ == "__main__":
    rospy.init_node("reward_node")
    rn = RewardNode()
    rospy.spin()
