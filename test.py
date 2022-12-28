import geopandas as gpd
import folium

# patial_buildings = gpd.read_file("./data/pluto.geojson")
# all_buildings_1 = gpd.read_file("./data/all_buildings.geojson")
# all_buildings_2 = gpd.read_file("./data/all_buildings/pluto.shp")
# new_all_buildings = gpd.read_file(
#     "./data/building/geo_export_730bc835-6635-46cf-b0c3-c5129b3bf7d4.shp"
# )
new_all_buildings = gpd.read_file("./data/new_all_buildings/new_all_buildings.shp")
# new_all_buildings.to_file("./data/new_all_buildings.geojson", driver="GeoJSON")
# Plot the map
m = new_all_buildings.explore()
folium.LayerControl().add_to(m)
m.save("./data/new_all_buildings.html")


# print("patial_buildings -> ", patial_buildings.shape)
# print("all_buildings_1 -> ", all_buildings_1.shape)
# print("all_buildings_2 -> ", all_buildings_2.shape)
# print("new_all_buildings -> ", new_all_buildings.shape)
# print("-----------------------------")
# print("all_buildings_1 info -> ", all_buildings_1.info())
# print("all_buildings_2 info -> ", all_buildings_2.info())
# print("new_all_buildings info -> ", new_all_buildings.info())
