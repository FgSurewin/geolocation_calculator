import time
import pyproj
import folium
import numpy as np
import geopandas as gpd
from pprint import pprint
import shapely.geometry as geom

"""
copyright: 2022
author: Jiawei Liu
"""


class GeolocationCalculator:
    """
    This class calculates the distance and the intersection point of a ray from a point of interest with the nearest building.
    
    """

    def __init__(self, file_path, target_crs=2263):
        """
        :param file_path: path to the file containing the buildings information with the extension "shp" or "geojson".
        :param target_crs: the target coordinate reference system (CRS) to which the buildings will be transformed.
            * 2263: NAD83 / New York Long Island (ftUS)
            * 4326: WGS 84 - longitude/latitude
        """
        self.buildings_gdf = gpd.read_file(file_path).to_crs(target_crs)
        self.target_crs = target_crs
        # Transform a Point object from CRS 4326 to CRS 2263
        self.transformer_ = pyproj.Transformer.from_crs(
            4326, self.target_crs, always_xy=True
        )
        # Transform a Point object from CRS 2263 to CRS 4326
        self.back_transformer_ = pyproj.Transformer.from_crs(
            self.target_crs, 4326, always_xy=True
        )

    def transform_(self, lat, lng):
        """
        This function transforms a point from CRS 4326 to CRS 2263
        :param lat: latitude
        :param lng: longitude
        """
        return geom.Point(self.transformer_.transform(float(lng), float(lat)))

    def back_transform_(self, point_object):
        """
        This function transforms a point from CRS 2263 to CRS 4326
        :param point_object: a Point object from shapely.geometry.Point class.
        """
        return self.back_transformer_.transform(point_object.x, point_object.y)

    def generate_ray(self, point, heading, distance=500):
        """
        This function generates a ray from a point of interest.
        :param point: a Point object from shapely.geometry.Point class.
        :param heading: the heading in degrees.
        :param distance: the distance in meter to be used to generate the ray from the point of interest.
        """
        # Create a point 500 meters away from the starting point in the desired direction of the ray.
        end_point = (
            point.x + distance * np.sin(np.deg2rad(heading)),
            point.y + distance * np.cos(np.deg2rad(heading)),
        )

        # Create a LineString object with the starting and end points
        ray = geom.LineString([point, end_point])

        return ray

    def generate_bbox(geom_obj):
        """
        This function generates a bounding box from a geometry object.
        :param geom_obj: a geometry object from shapely.geometry class.
        """
        bounds = geom_obj.bounds
        # The order is left_top, left_bottom, right_bottom, right_top
        bbox = geom.Polygon(
            [
                (bounds[0], bounds[1]),
                (bounds[0], bounds[3]),
                (bounds[2], bounds[3]),
                (bounds[2], bounds[1]),
            ]
        )
        return bbox

    def get_nearest_intersection(self, lat, lng, heading, distance=500):
        """
        This function finds the nearest intersection point of a ray from a point of interest with the nearest building.
        :param lat: latitude
        :param lng: longitude
        :param heading: the heading in degrees.
        :param distance: the distance in meter to be used to generate the ray from the point of interest.
        """
        # Check if the heading is within the range of 0 to 360
        if heading < 0:
            heading = 360 + heading
        elif heading > 360:
            heading = heading - 360

        # Initialize variables to store nearest intersection point and distance
        self.nearest_distance = float("inf")
        self.nearest_intersection = None

        # Initialize variables to store the point of interest and the ray
        self.geo_point = self.transform_(lat, lng)
        self.ray = self.generate_ray(self.geo_point, heading, distance)

        # Find all buildings that intersect with the ray
        self.candidate_buildings = (
            self.buildings_gdf[self.buildings_gdf.intersects(self.ray)]
            .copy()
            .reset_index(drop=True)
        )

        # Loop through candidate buildings and find nearest intersection point
        for index, row in self.candidate_buildings.iterrows():
            building = row.geometry
            intersections = self.ray.intersection(building)
            if intersections:
                if intersections.geom_type == "MultiLineString":
                    for line in intersections.geoms:
                        for point in line.coords:
                            # Convert the coordinates to a Point object
                            intersection_point = geom.Point(point)
                            distance = intersection_point.distance(self.geo_point)
                            if distance < self.nearest_distance:
                                self.nearest_distance = distance
                                self.nearest_intersection = intersection_point
                elif intersections.geom_type == "LineString":
                    for intersection in intersections.coords:
                        intersection_point = geom.Point(intersection)
                        distance = intersection_point.distance(self.geo_point)
                        if distance < self.nearest_distance:
                            self.nearest_distance = distance
                            self.nearest_intersection = intersection_point

        # Convert the nearest intersection point to latitude and longitude
        if self.nearest_intersection:
            self.nearest_intersection = self.back_transform_(self.nearest_intersection)

        return {
            "distance": self.nearest_distance,
            "intersection": self.nearest_intersection,
        }

    def plot_map(self, point_buffer=10):
        """
        This function plots the map with the ray and the nearest intersection point.
        :param point_buffer: the buffer size in meter to be used to plot the nearest intersection point.
        """
        # Convert the nearest intersection point to CRS 2263
        nearest_intersection_2263 = self.transform_(
            self.nearest_intersection[1], self.nearest_intersection[0]
        )

        # Generate geodataframes for the ray and the nearest intersection point
        ray_gdf = gpd.GeoDataFrame(
            dict(address="Test_ray", bbl=1, geometry=[self.ray]), crs=self.target_crs
        )
        intersection_point_df = gpd.GeoDataFrame(
            dict(
                address="Test_point",
                bbl=2,
                geometry=[nearest_intersection_2263.buffer(point_buffer)],
            ),
            crs=self.target_crs,
        )

        # Plot the map
        m = self.buildings_gdf.explore()
        ray_gdf.explore(m=m, color="red")
        intersection_point_df.explore(m=m, color="yellow")
        folium.LayerControl().add_to(m)

        return m


if __name__ == "__main__":
    # Calculate the computation time
    print("From rtree calculator")
    start = time.time()

    # Global variables
    # FILE_PATH = "./data/pluto.geojson"
    FILE_PATH = "./data/pluto_all_buildings_MN.geojson"
    POINT = (40.7665674, -73.9792634)
    # POINT = (40.74177025108547, -73.99407876316269)
    HEADING = 290

    # Initialize the class
    geolocation_calculator = GeolocationCalculator(FILE_PATH)

    # Find the nearest intersection point
    result = geolocation_calculator.get_nearest_intersection(
        lat=POINT[0], lng=POINT[1], heading=HEADING, distance=300
    )
    pprint(result)

    # Calculate the computation time
    end = time.time()
    print("Time taken: ", end - start)

    # Plot the map
    # my_map = geolocation_calculator.plot_map()

    # save as html
    # my_map.save("./result_1.html")

# 40.76661562871582, -73.97943755429039

# 40.76661559663138, -73.97943743843283
