import numpy as np
import math
from GPS import get_vector, set_origin, get_gps

# origin
latO = -35.3626697
lonO = 149.1640547

# new point GPS
latN = -35.11
lonN = 149.16
altN = 3.973
radius = 2

origin = set_origin(latO, lonO)

'''lats, longs = get_circle_coords(latN, lonN, origin)

print('done')'''
wp_pos = get_vector(origin, latN, lonN)

circle_x = []
circle_y = []
circle_lats = []
circle_longs = []

for i in range(0, 361):
    rad = math.radians(i)
    circle_x.append(wp_pos[0] + radius * math.cos(rad))
    circle_y.append(wp_pos[1] + radius * math.sin(rad))
    circle_point = np.array([circle_x[i], circle_y[i], wp_pos[2], 1])
    circle_point = np.transpose(circle_point)
    lat, lon, long2 = get_gps(origin, circle_point)
    circle_lats.append(lat)
    circle_longs.append(long2)

print('done')