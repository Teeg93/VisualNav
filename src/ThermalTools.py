import cv2
import numpy as np

def raw_to_thermal_frame(raw_frame, min, max):
    """
    Use the device configuration to scale the 16-bit image to an 8 bit image, given the min & max temperatures.

    :param raw_frame: 16 bit radiometric image
    :return: 8 bit temperature scaled image
    """

    if min >= max:
        print("Warning: Minimum temperature greater than or equal to max temperature. Setting min less than max")
        min = max - 1
        min_temp.set(max_temp.get() -1)

    # Get temperature range limits
    _dt = max - min

    # Equivalent to 255*( ( ( (x-20000) / 100) -_min ) / ( _max - _min) )
    # Much faster than floating point temperature operations
    temp = cv2.convertScaleAbs(raw_frame, alpha=255 / (100 * _dt), beta=-255 * (273 / _dt + min / _dt))
    return temp


def raw_pixel_to_deg_c(val):
    deg_k = val / 100
    deg_c = deg_k - 273.15
    return deg_c

def raw_pixel_to_deg_k(val):
    deg_k = val / 100
    return deg_k

def compute_thermal_bracket(frame, min_val=27300, max_val=37300, count_threshold=20):
    """
    Find an appropriate themal bracket by looking at the image histogram

    min_val defaults to 0 deg C
    max_val defaults to 100 deg C

    """
    count, bins = np.histogram(frame, bins=1000, range=(min_val, max_val))

    min_idx = np.min(np.where(count > count_threshold))
    max_idx = np.max(np.where(count > count_threshold))+1

    min_temp = raw_pixel_to_deg_c(bins[min_idx])
    max_temp = raw_pixel_to_deg_c(bins[max_idx])

    return min_temp, max_temp


class ThermalFlow:
    def __init__(self):
        self.previous_frame = None
        self.current_frame = None
    

    def pass_frame(self, frame):
        min, max = compute_thermal_bracket(frame)


