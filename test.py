import numpy.linalg
import numpy as np
import math
from GPS2 import get_vector, set_origin, get_gps

# origin
latO = -35.3626697
lonO = 149.1640547

# new point GPS
'''latN = -20.36265179
lonN = 145.16401228
altN = 3.973'''
latN = -35.36265179
lonN = 149.16401228
altN = 3.973

origin_fo = set_origin(latO, lonO)

vector_fn = get_vector(origin_fo, latN, lonN)

lat, long, long2 = get_gps(origin_fo, vector_fn)

error_lat = abs(latN) - abs(lat)
error_long = abs(lonN) - abs(long2)

print('Done')