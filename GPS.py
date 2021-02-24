import numpy.linalg as npl
import numpy as np
import math


# gets unit vector of v (np.array)
def normalise(v):
    v = v/npl.norm(v)
    return v


# gets the global cartesian coordinates (ECEF) of a gps point
def gps_to_cartesian(lat, long):
    #lat = int
    #long = int
    radius = 6378100
    # degrees to radians:
    lat = (lat / 180) * math.pi
    long = (long / 180) * math.pi
    # define vector in global cartesian coordinate frame
    x = radius * math.cos(lat) * math.cos(long)
    y = radius * math.cos(lat) * math.sin(long)
    z = radius * math.sin(lat)
    return x, y, z


# gets the gps coordinate in lat long, given global cartesian (ECEF) coordinates
def cartesian_to_gps(global_vec):
    radius = 6378100

    # inverse of that done above to find cartesian
    lat = math.asin(global_vec[2]/radius)
    long = math.asin(global_vec[1]/(radius*math.cos(lat)))
    long2 = math.acos(global_vec[0]/(radius*math.cos(lat)))

    # since inverse sin is used for long, the answer is 180 - ...
    lat = (lat / math.pi) * 180.0
    long = 180 - (long / math.pi) * 180.0
    long2 = (long2 / math.pi) * 180.0

    return lat, long2


def set_origin(lat, long):

    # gets the global cartesian coordinates (ECEF) of the point
    x, y, z = gps_to_cartesian(lat, long)
    r = np.array([x, y, z])  # vector in global cartesian

    # define rotation of new frame w.r.t origin of global frame
    z = np.array([0, 0, 1])
    zn = normalise(r)

    # taking cross products of the unit vectors defines the new x and y axis (xn and yn)
    xn = normalise(np.cross(z, zn))
    yn = normalise(np.cross(zn, xn))

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

    # vec_local is a np.array in homogeneous coordinates, so [x y z 1]
    vec_global = np.matmul(local_frame, vec_local)

    # due to the use of inverse trigonometric functions in cartesian_to_gps, the longitude
    lat, long = cartesian_to_gps(vec_global)

    return lat, long


# gets the coordinates of a circle about a given point lat long
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


def distance_between(lat1, long1, lat2, long2, origin):

    pos1 = get_vector(origin, lat1, long1)
    pos2 = get_vector(origin, lat2, long2)
    diff = pos2 - pos1
    distance = npl.norm(diff)

    return distance
