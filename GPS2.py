import numpy.linalg as npl
import numpy as np
import math


def normalise(v):
    v = v/npl.norm(v)
    return v


def gps_to_cartesian(lat, long):
    radius = 6378100
    lat = (lat / 180) * math.pi
    long = (long / 180) * math.pi
    # define vector from GPS origin to coordinate in cartesian
    x = radius * math.cos(lat) * math.cos(long)
    y = radius * math.cos(lat) * math.sin(long)
    z = radius * math.sin(lat)
    return x, y, z


def cartesian_to_gps(vector_fo):
    radius = 6378100

    lat = math.asin(vector_fo[2]/radius)
    long = math.asin(vector_fo[1]/(radius*math.cos(lat)))
    long2 = math.acos(vector_fo[0]/(radius*math.cos(lat)))

    lat = (lat / math.pi) * 180.0
    long = (long / math.pi) * 180.0
    long2 = (long2 / math.pi) * 180.0

    return lat, long, long2


def set_origin(lat, long):

    # get cartesian coordinates relative to Global origin
    x, y, z = gps_to_cartesian(lat, long)
    r = np.array([x, y, z])  # vector coordinates to point

    # define rotation of new frame w.r.t origin
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

    # Create Pose of new frame w.r.t origin (i.e frame Origin = fO)
    rot = np.c_[xn, yn, zn]
    origin_pose_fo = np.vstack([np.c_[rot, r], np.array([0, 0, 0, 1])])

    return origin_pose_fo


def get_vector(origin_pose_fo, lat, long):

    # convert to cartesian
    x, y, z = gps_to_cartesian(lat, long)
    p_fo = np.array([x, y, z, 1])
    p_fo = np.transpose(p_fo)

    # inverse of origin_pose_fo
    vector_fn = np.matmul(npl.inv(origin_pose_fo), p_fo)

    return vector_fn  # to return the values


def get_gps(origin_pose, vector_fn):

    vector_fo = np.matmul(origin_pose, vector_fn)

    lat, long, long2 = cartesian_to_gps(vector_fo)

    return lat, long, long2


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
        lat, lon, long2 = get_gps(origin, circle_point)
        circle_lats.append(lat)
        circle_longs.append(long2)

    return circle_lats, circle_longs


def distance_between(lat1, long1, lat2, long2, origin):

    pos1 = get_vector(origin, lat1, long1)
    pos2 = get_vector(origin, lat2, long2)
    diff = pos2 - pos1
    distance = npl.norm(diff)

    return distance
