from GPS import get_vector, set_origin, get_gps
import numpy as np
import math
import numpy.linalg as npl

# origin Warwick=52.38255, -1.56156, 0, 0
latO = 52.38255
lonO = -1.56156

local_frame = set_origin(latO, lonO)

'''vec_local = get_vector(local_frame, 52.38345, -1.56009)
lat, long = get_gps(local_frame, vec_local)'''

test_p_north = [100, -50, 500, -20]
test_p_east = [50, -100, 20, -500]
pos_lat = []
pos_lon = []

for i in range(len(test_p_east)):
    vec_local = np.array([test_p_north[i], test_p_east[i], 0, 1])
    lat, long = get_gps(local_frame, vec_local)
    pos_lat.append(lat)
    pos_lon.append(long)

x = []
y = []

for i in range(len(pos_lat)):
    vec_local = get_vector(local_frame, pos_lat[i], pos_lon[i])
    x.append(vec_local[0])
    y.append(vec_local[1])

err_x = []
err_y = []

for i in range(len(x)):
    err_x.append(test_p_north[i] - x[i])
    err_y.append(test_p_east[i] - y[i])

# new point GPS
latitudes = [52.38345, 52.38372, 52.38399, 52.38525, 52.38704, 52.37086]
longitudes = [-1.56009, -1.55861, -1.56436, -1.56274, -1.55420, -1.56892]

positions = []
distances = []
distances3 = []
latitudes2 = []
longitudes2 = []
for i in range(len(latitudes)):

    positions.append(get_vector(local_frame, latitudes[i], longitudes[i]))
    x = positions[i][0]
    y = positions[i][1]
    d = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
    distances.append(d)
    distances3.append(npl.norm(positions[i]))

norths = []
easts = []
errorlats = []
errorlongs = []

for i in range(len(positions)):
    '''norths.append(positions[i][0])
    easts.append(positions[i][1])'''

    lat, long = get_gps(local_frame, positions[i])
    latitudes2.append(lat)
    longitudes2.append(long)
    errorlats.append(latitudes[i] - lat)
    errorlongs.append(longitudes[i] - long)

print('Done')
