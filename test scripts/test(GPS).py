from GPS import get_vector, set_origin, get_gps

# origin
latO = -35.3626697
lonO = 149.1640547
altO = 3.954

# new point GPS
latN = -35.36265179
lonN = 149.16401228
altN = 3.973

local_frame = set_origin(latO, lonO)

vec_local = get_vector(local_frame, latN, lonN)

lat, long = get_gps(local_frame, vec_local)

print('Done')