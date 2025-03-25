import numpy as np
import scipy.optimize
from ctypes import *
import math
import cv2
import matplotlib.pyplot as plt

from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

EARTH_ANGULAR_RATE_DEGREES_PER_HOUR = 15.04108
EARTH_POLAR_RADIUS =      6356752
EARTH_EQUATORIAL_RADIUS = 6378137


def draw_text_with_background(img, text,
          font=cv2.FONT_HERSHEY_PLAIN,
          pos=(0, 0),
          font_scale=3,
          font_thickness=2,
          text_color=(0, 255, 0),
          bg_color=(0, 0, 0),
          bg_padding_x = 2,
          bg_padding_y = 2,
          ):

    x, y = pos
    bg_pos = x-bg_padding_x, y - bg_padding_y

    text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_w, text_h = text_size
    img = cv2.rectangle(img, bg_pos, (x + text_w + bg_padding_x * 2, y + text_h + bg_padding_y*2), bg_color, -1)
    img = cv2.putText(img, text, (x, y + text_h + font_scale - 1), font, font_scale, text_color, font_thickness)
    return img

def expand_bounding_box(detection, pixels):
    detection.bb_x = detection.bb_x - pixels
    detection.bb_y = detection.bb_y - pixels
    detection.bb_w = detection.bb_w + 2*pixels
    detection.bb_h = detection.bb_h + 2*pixels
    return detection

def to_C_double_array(dat):
    if isinstance(dat, np.ndarray) and dat.dtype == np.float64 and len(dat.shape) == 1:
        ret = dat.ctypes.data
    else:
        arr_t = c_double * len(dat)
        ret = arr_t(*dat)
    return ret


def to_C_float_array(dat):
    if isinstance(dat, np.ndarray) and dat.dtype == np.float32 and len(dat.shape) == 1:
        ret = dat.ctypes.data
    else:
        arr_t = c_float * len(dat)
        ret = arr_t(*dat)
    return ret

def to_C_int_array(dat):
    if isinstance(dat, np.ndarray) and dat.dtype == int and len(dat.shape) == 1:
        ret = dat.ctypes.data
    else:
        arr_t = c_int * len(dat)
        ret = arr_t(*dat)
    return ret

def transformNEDToAircraftFrame(roll, pitch, yaw):
    x = np.matrix(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
    C = buildRotationMatrix(roll, pitch, yaw)
    x_prime = np.matmul(C, x)
    return x_prime

def transformAircraftToNEDFrame(x, roll, pitch, yaw):
    C = buildRotationMatrix(roll, pitch, yaw)
    x_prime = np.matmul(np.linalg.inv(C), x)
    return x_prime

def transformAircraftToCameraFrame(x, camera_pitch_angle, camera_yaw_angle, camera_roll_angle=0, degrees=True):
    """
    Uses the camera configuration to rotate a vector from aircraft frame to camera frame

    :param x: the coordinate system to rotate
    :return: the rotated coordinate system
    """
    if not degrees:
        camera_pitch_angle = np.degrees(camera_pitch_angle)
        camera_yaw_angle = np.degrees(camera_yaw_angle)
        camera_roll_angle = np.degrees(camera_roll_angle)

    roll = float(camera_roll_angle)
    pitch = float(camera_pitch_angle)
    yaw = float(camera_yaw_angle)
    
    # Rotation matrix - body to camera frame 
    R_cam_body = buildRotationMatrix(roll, pitch, yaw)

    # Apply the rotation
    x_prime = np.matmul(R_cam_body, x)

    return x_prime

def transformCameraToAircraftFrame(x, camera_pitch_angle, camera_yaw_angle, camera_roll_angle=0, degrees=True):
    if not degrees:
        camera_pitch_angle = np.degrees(camera_pitch_angle)
        camera_roll_angle = np.degrees(camera_roll_angle)
        camera_yaw_angle = np.degrees(camera_yaw_angle)

    roll = float(camera_roll_angle)
    pitch = float(camera_pitch_angle)
    yaw = float(camera_yaw_angle)
    
    # Rotation matrix - body to camera frame
    R_cam_body = buildRotationMatrix(roll, pitch, yaw)
    
    # Transpose - we want to go from camera to body frame
    x_prime = np.matmul(R_cam_body.T, x)

    return x_prime

def transformCameraToImageFrame(x, dx, dy):
    """
    :param x: coordinate system to rotate
    :param dx: angular offset from centre of image (degrees)(positive right)
    :param dy: angular offset from centre of image (degrees)(positive down)
    :return:
    """
    C = buildRotationMatrix(0, -dy, dx)
    x_prime = np.matmul(C, x)
    return x_prime


def earthToLocalMatrix(lat, lng):
    """
    """
    lat = math.radians(lat)
    lng = math.radians(lng)
    from math import cos, sin
    C11 = -sin(lng)
    C12 = cos(lng)
    C13 = 0
    C21 = -sin(lat) * cos(lng)
    C22 = -sin(lat) * sin(lng)
    C23 = cos(lat)
    C31 = cos(lat) * cos(lng)
    C32 = cos(lat) * sin(lng)
    C33 = sin(lat)
    C = np.matrix(np.array([[C11, C12, C13], [C21, C22, C23], [C31, C32, C33]]))
    return C


def buildRotationMatrix(phi, theta, psi, degrees=True):
    """
    phi = roll (degrees)
    theta = pitch (degrees)
    psi = yaw (degrees)
    """
    if degrees:
        phi = math.radians(phi)
        theta = math.radians(theta)
        psi = math.radians(psi)
    from math import cos, sin
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


def DCMtoEuler(x):
    """
    Converts a DCM into its Euler angles (radians)

    :ret phi: roll
    :ret theta: pitch
    :ret psi: yaw
    """
    from math import atan2, asin
    phi = atan2(x[1, 2], x[2, 2])
    theta = -asin(x[0, 2])
    psi = atan2(x[0, 1], x[0, 0])
    return phi, theta, psi


def transformUnitVectorNEDtoENU(x):
    """
    Convert a NED vector into ENU coordinate frame
    """
    v = np.array([x[1], x[0], -x[2]])
    return v

def transformUnitVectorENUtoNED(x):
    """
    Convert a NED vector into NED coordinate frame
    """
    v = np.array([x[1], x[0], -x[2]])
    return v

def angleBetweenUnitVector(u,v):
    return np.degrees(np.arccos(np.dot(u,v)))

def polynomial_function(x, a, b, c, d):
    """
    Polynomial fitness function for star rendering
    """
    return a - (b * x ** 2.0) - (c * x ** 4.0) - (d * x ** 6.0)

def gaus2d(x=0, y=0, ux=0, uy=0, xx=1, yy=1):
    """
    2 dimensional gaussian
    :param ux: mean x value
    :param uy: mean y value
    :param xx: standard deviation (xx)
    :param yy: standard deviation (yy)
    """
    return 1. / (2. * np.pi * xx * yy) * np.exp(-((x - ux)**2. / (2. * xx**2.) + (y - uy)**2. / (2. * yy**2.)))


def gauss2d(xy, amp, x0, y0, a, b, c):
    """
    2 dimensional gaussian with non-zero covariance
    :param xy: x and y values
    :param amp: amplitude
    :param x0: x origin
    :param y0: y origin
    :param a: xx covariance
    :param b: xy covariance
    :param c: yy covariance
    :return: Likelihood at x,y
    """
    
    x, y = xy
    
    inner = c * (x - x0) ** 2
    inner += 2 * b * (x - x0) * (y - y0)
    inner += a * (y - y0) ** 2
    return amp * np.exp(-inner)

def gaussian_function(x, a, b):
    """
    Gaussian fitness function for star rendering
    """
    return (a / np.sqrt(2 * np.pi * b ** 2.0)) * np.exp(-x ** 2.0 / (2 * b ** 2.0))

def rotationMatrixToQuaternion(C):
    r = R.from_matrix(C)
    return r.as_quat()


def interpolate(rotation_mats, timestamps, n):
    """
    SLERP - generate n rotations between C1 and C2
    """

    interp_times = np.linspace(start=timestamps[0], stop=timestamps[-1], num=n)
    r = R.from_matrix(rotation_mats)
    slerp = Slerp(timestamps, r)

    interp_rots = slerp(interp_times)

    ret = []
    # Iterate through all except the first and last rotation matrix
    for r in interp_rots:
        ret.append(r.as_matrix())

    # Return timestamps and rotation matrices
    return interp_times, ret


def latLonToUnitVec(lat, lon, degrees=True):
    """
    Converts latitude and longitude to a unit vector (ECEF)
    :param lat:
    :param lon:
    :param degrees:
    :return:
    """

    if degrees:
        lat = np.radians(lat)
        lon = np.radians(lon)

    x = np.cos(lat) * np.cos(lon)
    y = np.cos(lat) * np.sin(lon)
    z = np.sin(lat)
    return np.array([x,y,z])

def azElToUnitVec(az, el, degrees=True):
    """
    Converts azimuth and elevation into a unit vector (NED)
    :param az: Azimuth
    :param el: Elevation
    :param degrees: Boolean - have you provided azimuth and elevation in degrees? (True=degrees, False=radians)
    :return: ndarray, size (3,)
    """
    if degrees:
        az = np.radians(az)
        el = np.radians(el)

    x = np.cos(az) * np.cos(el)
    y = np.sin(az) * np.cos(el)
    z = -np.sin(el)

    return np.array([x,y,z])


def plotPoints3D(srcPoints, dstPoints):
    """
    Plot 2 sets of 3 dimensional points and display it in a 3d graph
    """
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.scatter(srcPoints[:,0], srcPoints[:,1], srcPoints[:,2])
    ax.scatter(dstPoints[:,0], dstPoints[:,1], dstPoints[:,2])

    plt.show()

def plot_multiple_images(images_list, title=None, titles=None, dpi=300, show_axis=True):
    """
    Show multiple images on the same plot
    :param images_list: list of images (numpy ndarrays)
    :param title: Title for the plot
    :param titles: Title for each individual image - in a list, with same indexes as images_list
    """

    n = len(images_list)
    fig = plt.figure(figsize=(10,5),dpi=dpi)

    if n >= 3:
        cols = 3
    else:
        cols = 1
    rows = int(n / cols)
    rows += n % cols

    for i in range(n):
        ax = fig.add_subplot(rows, cols, i+1)

        if titles is not None:
            ax.set_title(titles[i])

        if not show_axis:
            ax.set_axis_off()

        im = plt.imshow(images_list[i])
        im.set_clim(vmin=0, vmax=255)

    if title is not None:
        fig.suptitle(title)

    plt.margins(tight=True)

    fig.subplots_adjust(right=0.8)
    cbar_ax_2 = fig.add_axes([0.85, 0.1, 0.03, 0.6])
    fig.colorbar(im, cax=cbar_ax_2)

    plt.show()

def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance in kilometers between two points 
    on the earth (specified in decimal degrees)
    """

    from math import radians, cos, sin, asin, sqrt
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
    from math import radians, degrees, cos, sin, atan2
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    dlon = lon2 - lon1

    y = sin(dlon) * cos(lat2)
    x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
    b = atan2(y, x)

    bearing = degrees(b)

    return bearing


def fit_sinusoid(tt, yy):
    '''Fit sin to the input time sequence, and return fitting parameters "amp", "omega", "phase", "offset", "freq", "period" and "fitfunc"'''
    tt = np.array(tt)
    yy = np.array(yy)
    ff = np.fft.fftfreq(len(tt), (tt[1]-tt[0]))   # assume uniform spacing
    Fyy = abs(np.fft.fft(yy))
    guess_freq = abs(ff[np.argmax(Fyy[1:])+1])   # excluding the zero frequency "peak", which is related to offset
    guess_amp = np.std(yy) * 2.**0.5
    guess_offset = np.mean(yy)
    guess = np.array([guess_amp, 2.*np.pi*guess_freq, 0., guess_offset])

    def sinfunc(t, A, w, p, c):  return A * np.sin(w*t + p) + c
    popt, pcov = scipy.optimize.curve_fit(sinfunc, tt, yy, p0=guess)
    A, w, p, c = popt
    f = w/(2.*np.pi)
    fitfunc = lambda t: A * np.sin(w*t + p) + c
    return {"amp": A, "omega": w, "phase": p, "offset": c, "freq": f, "period": 1./f, "fitfunc": fitfunc, "maxcov": np.max(pcov), "rawres": (guess,popt,pcov)}

def draw_cross(frame, position, line_width, length, color=(255,255,255)):
    vertical_line_point_1 = (int(position[0]), int(position[1] - length/2))
    vertical_line_point_2 = (int(position[0]), int(position[1] + length/2))
    horizontal_line_point_1 = (int(position[0] - length/2), int(position[1]))
    horizontal_line_point_2 = (int(position[0] + length/2), int(position[1]))

    frame = cv2.line(frame, vertical_line_point_1, vertical_line_point_2, color, line_width, cv2.LINE_AA)
    frame = cv2.line(frame, horizontal_line_point_1, horizontal_line_point_2, color, line_width, cv2.LINE_AA)
    return frame

def euler_rate_to_body_rate(phi, theta, psi, phidot, thetadot, psidot):
    from numpy import sin, cos
    p = phidot - sin(theta)*psidot
    q = cos(phi)*thetadot + sin(phi)*cos(theta)*psidot
    r = -sin(phi)*thetadot + cos(phi)*cos(theta)*psidot
    return np.array([p,q,r])

def body_rate_to_euler_rate(phi, theta, psi, p, q, r):
    from numpy import sin, cos, tan
    phidot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r
    thetadot = cos(phi)*q - sin(phi)*r
    psidot = sin(phi)*(1/cos(theta))*q + cos(phi)*(1/cos(theta))*r
    return np.array([phidot, thetadot, psidot])

def draw_locations_on_map(centre_point, latitudes, longitudes, idx1=0, idx2=0, directory=None, groundtruth=None):
    import folium
    import os

    g_map = folium.Map(location = centre_point)
    folium.Marker(centre_point, popup='Point').add_to(g_map)
    if groundtruth is not None:
        folium.Marker(groundtruth, popup='Point', color='red').add_to(g_map)

    for lat, lon in zip(latitudes, longitudes):
        folium.CircleMarker((lat,lon), radius=3, color='green', fill=True).add_to(g_map)

    if directory is not None:
        output_path = os.path.join(directory, f"map_{idx1}_{idx2}.html")
    else:
        output_path = f"map_{idx1}_{idx2}.html"

    print("Saving map to: ", output_path)
    g_map.save(output_path)



def calculate_contour_distance(contour1, contour2): 
    x1, y1, w1, h1 = cv2.boundingRect(contour1)
    c_x1 = x1 + w1/2
    c_y1 = y1 + h1/2

    x2, y2, w2, h2 = cv2.boundingRect(contour2)
    c_x2 = x2 + w2/2
    c_y2 = y2 + h2/2

    return max(abs(c_x1 - c_x2) - (w1 + w2)/2, abs(c_y1 - c_y2) - (h1 + h2)/2)

def merge_contours(contour1, contour2):
    return np.concatenate((contour1, contour2), axis=0)

def agglomerative_cluster(contours, threshold_distance=40.0):
    current_contours = contours
    while len(current_contours) > 1:
        min_distance = None
        min_coordinate = None

        for x in range(len(current_contours)-1):
            for y in range(x+1, len(current_contours)):
                distance = calculate_contour_distance(current_contours[x], current_contours[y])
                if min_distance is None:
                    min_distance = distance
                    min_coordinate = (x, y)
                elif distance < min_distance:
                    min_distance = distance
                    min_coordinate = (x, y)

        if min_distance < threshold_distance:
            index1, index2 = min_coordinate
            current_contours[index1] = merge_contours(current_contours[index1], current_contours[index2])
            del current_contours[index2]
        else: 
            break

    return current_contours

def amplify_image(frame, amplification):
    frame = frame.copy()
    amplified_frame = frame.astype(float) * float(amplification)
    amplified_frame[amplified_frame > 255] = 255
    amplified_frame = amplified_frame.astype(np.uint8)
    return amplified_frame

def gaussian_kernel(l=5, sig=1.):
    """\
    creates gaussian kernel with side length `l` and a sigma of `sig`
    """
    ax = np.linspace(-(l - 1) / 2., (l - 1) / 2., l)
    gauss = np.exp(-0.5 * np.square(ax) / np.square(sig))
    kernel = np.outer(gauss, gauss)
    return kernel / np.sum(kernel)



def getEarthRadiusAtLatitude(lat):

    lat = np.radians(lat)
    a = (EARTH_EQUATORIAL_RADIUS**2.0 * np.cos(lat))**2.0
    b = (EARTH_POLAR_RADIUS**2.0 * np.sin(lat))**2.0
    c = (EARTH_EQUATORIAL_RADIUS * np.cos(lat))**2.0
    d = (EARTH_POLAR_RADIUS * np.sin(lat))**2.0

    R = np.sqrt( (a + b) / (c + d) )
    return R

def rotateECEFtoNED(x, lat, lon, degrees=True):
    from numpy import sin, cos
    if degrees:
        lat = np.radians(lat)
        lon = np.radians(lon)

    C_11 = -sin(lat) * cos(lon)
    C_12 = -sin(lat) * sin(lon)
    C_13 = cos(lat)
    C_21 = -sin(lon)
    C_22 = cos(lon)
    C_23 = 0
    C_31 = -cos(lat) * cos(lon)
    C_32 = -cos(lat) * sin(lon)
    C_33 = -sin(lat)

    C = np.array([
        [C_11, C_12, C_13],
        [C_21, C_22, C_23],
        [C_31, C_32, C_33],
    ])

    x_prime = np.matmul(C, x)
    return x_prime

def rotateNEDtoECEF(x, lat, lon, degrees=True):
    from numpy import sin, cos, radians

    if degrees:
        lat = radians(lat)
        lon = radians(lon)

    C_11 = -sin(lat) * cos(lon)
    C_12 = -sin(lat) * sin(lon)
    C_13 = cos(lat)
    C_21 = -sin(lon)
    C_22 = cos(lon)
    C_23 = 0
    C_31 = -cos(lat) * cos(lon)
    C_32 = -cos(lat) * sin(lon)
    C_33 = -sin(lat)

    C = np.array([
        [C_11, C_12, C_13],
        [C_21, C_22, C_23],
        [C_31, C_32, C_33],
    ])

    x_prime = np.matmul(C.T, x)
    return x_prime
