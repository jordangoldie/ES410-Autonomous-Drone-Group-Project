import numpy.linalg as npl
import numpy as np
import math


def normalise(v):
    v = v/np.linalg.norm(v)
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


# some issue here
def cartesian_to_gps(vector_fo):
    radius = 6378100

    lat = math.asin(vector_fo[2]/radius)
    long = math.asin(vector_fo[1]/(radius*math.cos(lat)))
    long2 = math.acos(vector_fo[0]/(radius*math.cos(lat)))

    lat = (lat / math.pi) * 180
    long = (long / math.pi) * 180
    long2 = (long2 / math.pi) * 180

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


def get_vector(origin_pose_fo, lat, long, alt):

    # convert to cartesian
    x, y, z = gps_to_cartesian(lat, long)
    p = np.array([x, y, alt, 1])
    p = np.transpose(p)

    # inverse of origin_pose_fo
    vector_fn = np.matmul(npl.inv(origin_pose_fo), p)

    return vector_fn  # to return the values


# this function isn't yet working perfect
def get_gps(origin_pose, vector_fn):

    vector_fo = np.matmul(origin_pose, vector_fn)

    lat, long, long2 = cartesian_to_gps(vector_fo)

    return lat, long, long2

