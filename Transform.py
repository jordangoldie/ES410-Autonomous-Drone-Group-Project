import numpy.linalg as npl
import numpy as np
import math


def normalise(v):
    v = v/np.linalg.norm(v)
    return v


# origin
latO = (-35.3626697/180) * math.pi
lonO = (149.1640547/180) * math.pi
altO = 3.954

# new point GPS
latN = (-35.36265179/180) * math.pi
lonN = (149.16401228/180) * math.pi
altN = 3.973

alt_diff = altN - altO

# define vector to new frame
radius = 6378100

x = radius * math.cos(latO) * math.cos(lonO)
y = radius * math.cos(latO) * math.sin(lonO)
z = radius * math.sin(latO)

R = np.array([x, y, z])

# define rotation of new frame w.r.t origin
Z = np.array([0, 0, 1])
Zn = normalise(R)
Xn = np.cross(Z, Zn)
Xn = normalise(Xn)
Yn = np.cross(Zn, Xn)
Yn = normalise(Yn)
Zn = np.transpose(Zn)
Yn = np.transpose(Yn)
Xn = np.transpose(Xn)

# make the 3x3 rotation matrix
Rot = np.c_[Xn, Yn, Zn]

# Create roto-translation aka Pose of new frame w.r.t origin
R = np.transpose(R)
uTn = np.vstack([np.c_[Rot, R], np.array([0, 0, 0, 1])])

# define new point
x = radius * math.cos(latN) * math.cos(lonN)
y = radius * math.cos(latN) * math.sin(lonN)
z = radius * math.sin(latN)
p = np.array([x, y, z])
p = np.transpose(p)

# define pose of point w.r.t origin
uTp = np.vstack([np.c_[Rot, p], np.array([0, 0, 0, 1])])

# define pose of point w.r.t new frame
nTp = np.matmul(npl.inv(uTn), uTp)
print(f'point p w.r.t new frame: {nTp}')

UnityX = nTp[0, 3]
UnityY = nTp[1,3]
UnityZ = nTp[2,3]

Unity_Coordinates = f'{UnityX},{UnityY}, {UnityZ}'

