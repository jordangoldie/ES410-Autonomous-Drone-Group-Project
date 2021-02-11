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
def cartesian_to_gps(x, y, z):
    radius = 6378100
    lat = math.asin(z/radius)
    long = math.asin(y/(radius*math.cos(lat)))
    long2 = math.acos(x/(radius*math.cos(lat)))

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

    # make the 3x3 rotation matrix which describes the orientation of our new coordinate frame
    rot = np.c_[xn, yn, zn]

    # Create Pose of new frame w.r.t origin (i.e frame Origin = fO)
    origin_pose_fo = np.vstack([np.c_[rot, r], np.array([0, 0, 0, 1])])
    return origin_pose_fo, rot


def get_vector(origin_pose, rot, lat, long, alt):

    # convert to cartesian
    x, y, z = gps_to_cartesian(lat, long)
    p = np.array([x, y, z])
    p = np.transpose(p)

    # define pose of point w.r.t origin
    point_pose_fo = np.vstack([np.c_[rot, p], np.array([0, 0, 0, 1])])

    # define pose of point w.r.t new frame
    point_pose_fn = np.matmul(npl.inv(origin_pose), point_pose_fo)

    # output x y z as string for unity
    unity_x = point_pose_fn[0, 3]
    unity_y = point_pose_fn[1, 3]
    unity_z = alt

    # CSV string
    unity_coord = f'{unity_x},{unity_y}, {unity_z}'

    return unity_x, unity_y, unity_z  # to return the values
    # return unity_coord


# this function isn't yet working perfect
def get_gps(origin_pose, rot, x, y, z):

    p = np.array([x, y, z])
    point_pose_fn = np.vstack([np.c_[rot, p], np.array([0, 0, 0, 1])])
    point_pose_fo = np.matmul(origin_pose, point_pose_fn)

    # output global x y z
    x = point_pose_fo[0, 3]
    y = point_pose_fo[1, 3]
    z = point_pose_fo[2, 3]

    lat, long, long2 = cartesian_to_gps(x, y, z)

    return lat, long, long2
