import numpy as np
import numpy.linalg as npl
import math


def normalise(v):
    v = v/np.linalg.norm(v)
    return v


