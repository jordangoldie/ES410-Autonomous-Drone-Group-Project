import numpy.linalg
import numpy as np
import math
from GPS import get_vector, set_origin, get_gps

# origin
latO = -35.3626697
lonO = 149.1640547
altO = 3.954

# new point GPS
latN = -35.36265179
lonN = 149.16401228
altN = 3.973

origin, rot = set_origin(latO, lonO)

x, y, z = get_vector(origin, rot, latN, lonN, altN)

lat, long, long2 = get_gps(origin, rot, x, y, z)

print('Done')