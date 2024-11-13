import cv2
import numpy as np

color = np.random.randint(0, 255, (100, 3))

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
    def __init__(self, redetection_interval=5, maximum_track_len=10):
        """

        :param redetection_interval: Number of frames after which points will be recalculated
        """

        self.previous_frame = None
        self.current_frame = None
        self.frame_idx = 0

        self.redetection_interval = redetection_interval
        self.maximum_track_len = maximum_track_len

        self.tracks = []
    
        self.feature_params = dict( maxCorners = 300,
                                    qualityLevel = 0.3,
                                    minDistance = 7,
                                    blockSize = 7 )
        
        self.lk_params = dict( winSize  = (15, 15),
                               maxLevel = 2,
                               criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    def pass_frame(self, frame):
        img = None
        _min_temp, _max_temp = compute_thermal_bracket(frame)
        frame_8_bit = raw_to_thermal_frame(frame, _min_temp, _max_temp)

        # Create a 3-channel colour frame for display purposes
        display_frame = np.reshape(frame_8_bit, (frame_8_bit.shape[0], frame_8_bit.shape[1], 1))
        display_frame = cv2.cvtColor(display_frame, cv2.COLOR_GRAY2BGR)

        if len(self.tracks) > 0:
            img0, img1 = self.previous_frame, frame_8_bit

            p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2)
            p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **self.lk_params)
            p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **self.lk_params)
            d = abs(p0-p0r).reshape(-1, 2).max(-1)
            good = d < 1
            new_tracks = []
            for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
                if not good_flag:
                    continue
                tr.append((x, y))
                if len(tr) > self.maximum_track_len:
                    del tr[0]
                new_tracks.append(tr)
                cv2.circle(display_frame, (int(x), int(y)), 2, (0, 255, 255), -1)
            self.tracks = new_tracks
            cv2.polylines(display_frame, [np.int32(tr) for tr in self.tracks], False, (0, 255, 0))

        if self.frame_idx % self.redetection_interval == 0:
        # Ignore temperatures that are clipping
            mask = cv2.inRange(frame_8_bit, 1, 254)

            # Ignore existing tracking points
            for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                cv2.circle(mask, (x, y), 5, 0, -1)
            
            p = cv2.goodFeaturesToTrack(frame_8_bit, mask=mask, **self.feature_params)

            if p is not None:
                for x, y in np.float32(p).reshape(-1, 2):
                    self.tracks.append([(x, y)])
        
        self.previous_frame = frame_8_bit
        return display_frame
            


        """
        if self.previous_frame is None:
            self.p0 = cv2.goodFeaturesToTrack(frame_8_bit, mask=mask, **self.feature_params)
            self.previous_frame = frame_8_bit

        else:
            self.p1, st, err = cv2.calcOpticalFlowPyrLK(self.previous_frame, frame_8_bit, self.p0, None, **self.lk_params)
            self.previous_frame = frame_8_bit
            if self.p1 is not None:
                good_new = self.p1[st==1]
                good_old = self.p0[st==1]
            
            frame_8_bit = np.reshape(frame_8_bit, (frame_8_bit.shape[0], frame_8_bit.shape[1], 1))
            frame_8_bit = cv2.cvtColor(frame_8_bit, cv2.COLOR_GRAY2BGR)
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel()
                c, d = old.ravel()

                mask = np.zeros_like(frame_8_bit)
                mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
                frame_display = cv2.circle(frame_8_bit, (int(a), int(b)), 5, color[i].tolist(), -1)

            img = cv2.add(frame_display, mask)

            self.p0 = good_new.reshape(-1, 1, 2)
        """
        
        if img is not None:
            return img
        else:
            return frame_8_bit
        









