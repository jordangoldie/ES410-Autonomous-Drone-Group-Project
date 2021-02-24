from GPS import get_vector, set_origin, get_gps

# origin Warwick=-35.36355729, 149.16460797, 0, 0
latO = -35.36355729
lonO = 149.16460797

# new point GPS
latN = -35.36265179
lonN = 149.16401228

local_frame = set_origin(latO, lonO)

vec_local = get_vector(local_frame, latN, lonN)

lat, long = get_gps(local_frame, vec_local)

print('Done')