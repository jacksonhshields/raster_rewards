#!/usr/bin/env python3


import numpy as np
from habitat_modelling.utils.raster_utils import extract_geotransform, get_pixels_from_lla, get_pixels_from_lla
import shapely
import utm


class RewardMap:
    def __init__(self, raster_path):
        self.reward_raster, _, _ = extract_geotransform(raster_path)
        self.geotransform = self.reward_raster.GetGeoTransform()
        self.projection = self.reward_raster.GetProjection()

        self.use_utm = True if 'UTM' in self.projection else False

        self.origin = [self.geotransform[0], # minimum x value
                       self.geotransform[3] + self.reward_raster.RasterYSize * self.geotransform[5]]  # max y value + num_y_pixels * pixel_height


        self.bounds_geo = shapely.geometry.box(self.origin[0],  # Minimum x value
                                               self.origin[1],  # Minimum y vale
                                               self.origin[0] + self.reward_raster.RasterXSize*self.geotransform[1],  # Max x value = origin + num_x_pixels * pixel_width
                                               self.geotransform[3])  # Max y value
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
            xy = list(utm.from_latlon(point[0], point[1]))[:2]
            return np.array(xy)
        else:
            return point

    def get_bounds(self, geo):
        if geo:
            return self.bounds_geo.bounds
        else:
            return self.bounds_local.bounds

    def get_point_reward(self, point, geo):
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

    def get_vector_reward(self, point):
        pass

    def show(self):
        plt.imshow(self.reward_raster.GetRasterBand(1).ReadAsArray(), cmap="jet")

    def create_plot_map(self, fig=None, ax=None):
        inproj = osr.SpatialReference()
        inproj.ImportFromWkt(self.projection)
        projcs = inproj.GetAuthorityCode('PROJCS')
        projection_ccrs = ccrs.epsg(projcs)
        subplot_kw = dict(projection=projection_ccrs)

        if fig is None and ax is None:
            fig, ax = plt.subplots(figsize=(9, 9), subplot_kw=subplot_kw)
            ax.set_title('Reward Map', fontsize=14)
        elif (fig is None and ax is not None) or (fig is not None and ax is None):
            warnings.warn("Either specify fig and ax or neither, overwriting given fig/ax")
            fig, ax = plt.subplots(figsize=(9, 9), subplot_kw=subplot_kw)
            ax.set_title('Reward Map', fontsize=14)

        extent = (self.geotransform[0], self.geotransform[0] + self.reward_raster.RasterXSize * self.geotransform[1],
                  self.geotransform[3] + self.reward_raster.RasterYSize * self.geotransform[5], self.geotransform[3])

        ax.imshow(self.reward_raster.GetRasterBand(1).ReadAsArray(), extent=extent,
                        origin='upper', cmap="viridis")

        return fig, ax

class RewardNode:
    def __init__(self):
        self.reward_raster_path = rospy.get_param("reward_raster_path", "")
        self.reward_map = RewardMap(self.reward_raster_path)
        
        self.reward_service = rospy.Service('get_reward', RasterPointReward, self.rewardServiceCallback)
    
    def self.rewardServiceCallback(self, req):
        point = np.array([req.latitude, req.longitude])
        
        res = RasterPointRewardResponse()
        geopoint = reward_map.llh_to_geo(point=point)
        res.reward = reward_map.get_point_reward(point=geopoint, geo=True)
        
        return res

if __name__ == "__main__":
    rospy.init_node("reward_node")
    rn = RewardNode()
    rospy.spin()
