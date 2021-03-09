import numpy.linalg as npl
import numpy as np
import math


# gets unit vector of v (np.array)
def normalise(v):
    v = v/npl.norm(v)
    return v


# gets the global cartesian coordinates (ECEF) of a gps point
def gps_to_cartesian(lat, long):
    # radius of the earth
    radius = 6371000
    # convert coordinates from degrees to radians:
    lat = (lat / 180) * math.pi
    long = (long / 180) * math.pi
    # define vector in global cartesian coordinate frame
    x = radius * math.cos(lat) * math.cos(long)
    y = radius * math.cos(lat) * math.sin(long)
    z = radius * math.sin(lat)
    return x, y, z


# gets the gps coordinate in lat long, given global cartesian (ECEF) coordinates
def cartesian_to_gps(global_vec):
    radius = 6371000

    # inverse of that done above to find cartesian
    lat = math.asin(global_vec[2]/radius)
    long = math.atan2(global_vec[1], global_vec[0])

    # convert to degrees from radians
    lat = math.degrees(lat)
    long = math.degrees(long)

    return lat, long


def set_origin(lat, long):

    # gets the global cartesian coordinates (ECEF) of the point
    x, y, z = gps_to_cartesian(lat, long)
    r = np.array([x, y, z])  # vector in global cartesian

    # define rotation of new frame w.r.t origin of global frame
    # find new up axis by normalising vector R in ecef global cartesian
    z = np.array([0, 0, 1])
    zn = normalise(r)  # up

    # taking cross products of the unit vectors defines the new x and y axis (xn and yn)
    yn = normalise(np.cross(z, zn))  # east
    xn = normalise(np.cross(zn, yn))  # north

    # transpose all vectors to be appended in rotation
    zn = np.transpose(zn)
    yn = np.transpose(yn)
    xn = np.transpose(xn)
    r = np.transpose(r)

    # defines the pose of the new local cartesian coordinate frame w.r.t the global cartesian frame (ECEF)
    rot = np.c_[xn, yn, zn]
    local_frame = np.vstack([np.c_[rot, r], np.array([0, 0, 0, 1])])

    # returns the pose which defines the local cartesian coordinate frame
    return local_frame


# given the pose of the local cartesian coordinate frame, any GPS coordinate is given as a vector in this local frame
def get_vector(local_frame, lat, long):

    # convert the new gps point to global cartesian (ECEF)
    x, y, z = gps_to_cartesian(lat, long)
    vec_global = np.array([x, y, z, 1])
    vec_global = np.transpose(vec_global)

    # uses the inverse of the local frame pose to
    vec_local = np.matmul(npl.inv(local_frame), vec_global)

    return vec_local  # to return the values


# gets the gps coordinates of a point (vec_local) defined in the local cartesian frame
def get_gps(local_frame, vec_local):

    # if vector accidentally given as direction, correct to homogeneous position
    if vec_local[3] == 0:
        vec_local[3] = 1

    # length of the local vector north, east
    vec_mag = math.sqrt(math.pow(vec_local[0], 2) + math.pow(vec_local[1], 2))

    # error based on distance from origin
    err = math.sqrt(math.pow(6371000, 2) - math.pow(vec_mag, 2)) - 6371000

    # add error to the the z to account for curvature of the earth
    vec_local[2] = err

    # vec_local is a np.array in homogeneous coordinates, so [x y z 1]
    vec_global = np.matmul(local_frame, vec_local)

    # due to the use of inverse trigonometric functions in cartesian_to_gps, the longitude
    lat, long = cartesian_to_gps(vec_global)

    return lat, long

# gets the GPS coordinates of a circle about a given point lat long
def get_circle_coords(lat, long, origin):
    radius = 2
    wp_pos = get_vector(origin, lat, long)

    circle_x = []
    circle_y = []
    circle_lats = []
    circle_longs = []

    for i in range(0, 361):
        circle_x.append(wp_pos[0] + radius * math.cos(i))
        circle_y.append(wp_pos[1] + radius * math.sin(i))
        circle_point = np.array([circle_x[i], circle_y[i], wp_pos[2], 1])
        circle_point = np.transpose(circle_point)
        lat, lon, = get_gps(origin, circle_point)
        circle_lats.append(lat)
        circle_longs.append(lon)

    return circle_lats, circle_longs


# calculates the distance between two lat/long GPS positions
def distance_between(lat1, long1, lat2, long2, origin):

    pos1 = get_vector(origin, lat1, long1)
    pos2 = get_vector(origin, lat2, long2)
    diff = pos2 - pos1
    distance = npl.norm(diff)

    return distance
