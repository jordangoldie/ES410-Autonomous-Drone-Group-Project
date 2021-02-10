import numpy.linalg as npl
import numpy as np
import math
from normalise import normalise


def set_origin(lat, long):
    # origin
    lat = (lat / 180) * math.pi
    long = (long / 180) * math.pi
    radius = 6378100

    # define vector from GPS origin to coordinate in cartesian
    x = radius * math.cos(lat) * math.cos(long)
    y = radius * math.cos(lat) * math.sin(long)
    z = radius * math.sin(lat)
    r = np.array([x, y, z])

    # define rotation of new frame w.r.t origin
    z = np.array([0, 0, 1])
    zn = normalise(r)
    xn = np.cross(z, zn)
    xn = normalise(xn)
    yn = np.cross(zn, xn)
    yn = normalise(yn)
    zn = np.transpose(zn)
    yn = np.transpose(yn)
    xn = np.transpose(xn)

    # make the 3x3 rotation matrix
    rot = np.c_[xn, yn, zn]

    # Create roto-translation aka Pose of new frame w.r.t origin
    r = np.transpose(r)
    origin_posefo = np.vstack([np.c_[rot, r], np.array([0, 0, 0, 1])])

    return origin_posefo, rot


def get_vector(origin_pose, rot, lat, long, alt):
    radius = 6378100
    lat = (lat / 180) * math.pi
    long = (long / 180) * math.pi

    # define cartesian vector with GPS origin
    x = radius * math.cos(lat) * math.cos(long)
    y = radius * math.cos(lat) * math.sin(long)
    z = radius * math.sin(lat)
    p = np.array([x, y, z])
    p = np.transpose(p)

    # define pose of point w.r.t origin
    point_posefo = np.vstack([np.c_[rot, p], np.array([0, 0, 0, 1])])

    # define pose of point w.r.t new frame
    point_posefn = np.matmul(npl.inv(origin_pose), point_posefo)

    unity_x = point_posefn[0, 3]
    unity_y = point_posefn[1, 3]
    unity_z = alt

    unity_coord = f'{unity_x},{unity_y}, {unity_z}'

    return unity_coord
