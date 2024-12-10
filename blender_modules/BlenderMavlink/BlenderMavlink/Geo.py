import numpy as np
from math import *

def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance in kilometers between two points 
    on the earth (specified in decimal degrees)
    """

    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371 # Radius of earth in kilometers
    return c * r

def bearing(lat1, lon1, lat2, lon2):
    """
    Calculate the bearing from point 1 to point 2
    """
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    dlon = lon2 - lon1

    y = sin(dlon) * cos(lat2)
    x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
    b = atan2(y, x)

    bearing = degrees(b)

    return bearing


def get_xy_offset_from_origin(origin_lat, origin_lon, target_lat, target_lon):

    target_range = haversine(origin_lat, origin_lon, target_lat, target_lon) * 1000
    target_bearing = bearing(origin_lat, origin_lon, target_lat, target_lon)

    offset_x = target_range * sin(radians(target_bearing))
    offset_y = target_range * cos(radians(target_bearing))

    return offset_x, offset_y


def rotation_matrix(phi, theta, psi, degrees=True):
    """
    phi = roll (degrees)
    theta = pitch (degrees)
    psi = yaw (degrees)
    """
    if degrees:
        phi = radians(phi)
        theta = radians(theta)
        psi = radians(psi)

    C11 = cos(theta) * cos(psi)
    C12 = cos(theta) * sin(psi)
    C13 = -sin(theta)
    C21 = -cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi)
    C22 = cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi)
    C23 = sin(phi) * cos(theta)
    C31 = sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi)
    C32 = -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi)
    C33 = cos(phi) * cos(theta)
    C = np.matrix(np.array([[C11, C12, C13], [C21, C22, C23], [C31, C32, C33]]))
    return C


def dcm_to_euler_angle(x):
    """
    Converts a DCM into its Euler angles (radians)

    :ret phi: roll
    :ret theta: pitch
    :ret psi: yaw
    """
    phi = atan2(x[1, 2], x[2, 2])
    theta = -asin(x[0, 2])
    psi = atan2(x[0, 1], x[0, 0])
    return phi, theta, psi


#def cv_to_blender_camera_euler_angles(x, y, z):





