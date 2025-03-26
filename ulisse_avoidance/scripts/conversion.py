from pyproj import Proj, Transformer

# Define centroid (can be manually set or passed as an argument)
CENTROID_LAT = 44.0939844  # Example latitude (centroid)
CENTROID_LON = 9.8627768   # Example longitude (centroid)
CENTROID_ALT = 0.0         # Example altitude (centroid)

# Create UTM and NED transformers using pyproj
def get_utm_zone(lat, lon):
    zone = int((lon + 180) / 6) + 1
    return f"epsg:326{zone}" if lat >= 0 else f"epsg:327{zone}"  # EPSG codes for UTM

# Use absolute UTM coordinates for the centroid
utm_proj = Proj(get_utm_zone(CENTROID_LAT, CENTROID_LON))  # UTM projection
wgs84_proj = Proj("epsg:4326")  # WGS84 projection
utm_transformer = Transformer.from_proj(wgs84_proj, utm_proj, always_xy=True)
utm_reverse_transformer = Transformer.from_proj(utm_proj, wgs84_proj, always_xy=True)

# UTM conversion: Convert lat/lon to UTM (absolute coordinates)
def latlong_to_utm(lat, lon):
    x, y = utm_transformer.transform(lon, lat)  # Convert lat/lon to absolute UTM
    return x, y  # Return absolute UTM coordinates

# UTM to lat/lon conversion: Convert UTM (x, y) to lat/lon
def utm_to_latlong(x, y):
    lon, lat = utm_reverse_transformer.transform(x, y)
    return lat, lon

# NED conversion: Convert lat/lon to NED (relative to centroid)
def latlong_to_ned(lat, lon):
    centroid_x, centroid_y = latlong_to_utm(CENTROID_LAT, CENTROID_LON)  # Centroid's absolute UTM
    x, y = latlong_to_utm(lat, lon)  # Convert lat/lon to absolute UTM

    # Calculate NED (relative to centroid)
    return x - centroid_x, y - centroid_y

# NED to lat/lon conversion: Convert NED to lat/lon (relative to centroid)
def ned_to_latlong(x, y):
    centroid_x, centroid_y = latlong_to_utm(CENTROID_LAT, CENTROID_LON)  # Centroid's absolute UTM

    # Convert NED back to absolute UTM
    abs_x = x + centroid_x
    abs_y = y + centroid_y

    # Convert absolute UTM to lat/lon
    lat, lon = utm_to_latlong(abs_x, abs_y)
    return lat, lon
