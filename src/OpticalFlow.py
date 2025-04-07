import cv2
import numpy as np
import time

class Track:
    def __init__(self):
        pass

class OpticalFlow:
    def __init__(self, redetection_interval=5, maximum_track_len=3, point_mask_radius=5, minimum_tracks=50):
        """

        :param redetection_interval: Number of frames after which points will be recalculated
        """

        self.previous_frame = None
        self.current_frame = None
        self.frame_idx = 0

        self.redetection_interval = redetection_interval
        self.maximum_track_len = maximum_track_len
        self.point_mask_radius=point_mask_radius
        self.minimum_num_tracks = minimum_tracks

        self.tracks = []

        self.feature_params = dict(maxCorners=20,
                                   qualityLevel=0.05,
                                   minDistance=point_mask_radius,
                                   blockSize=7)

        self.lk_params = dict(winSize=(15, 15),
                              maxLevel=2,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    def get_current_frame_index(self):
        return self.frame_idx

    def pass_frame(self, frame, R, agl):
        if frame is None:
            return


        self.frame_idx += 1
        display_frame = frame.copy()

        # Convert to grayscale
        if len(frame.shape) == 3 and frame.shape[2] >= 3:
            frame_grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Already single channel
        else:
            frame_grayscale = frame

        now = time.time()

        if len(self.tracks) > 0:
            img0, img1 = self.previous_frame, frame_grayscale

            p0 = np.float32([tr[-1][0:2] for tr in self.tracks]).reshape(-1, 1, 2)
            p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **self.lk_params)
            p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **self.lk_params)
            d = abs(p0 - p0r).reshape(-1, 2).max(-1)
            good = d < 1
            new_tracks = []
            for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
                if not good_flag:
                    continue
                tr.append((x, y, R, agl, now, self.frame_idx))
                if len(tr) > self.maximum_track_len:
                    del tr[0]
                new_tracks.append(tr)
                cv2.circle(display_frame, (int(x), int(y)), 4, (0, 255, 255), -1)
            

            self.tracks = new_tracks

            polylines = [[(t[0], t[1]) for t in tr] for tr in self.tracks ]
            polylines = [np.int32(t) for t in polylines]
            cv2.polylines(display_frame, polylines, False, (0, 255, 0), thickness=2)

        if (self.frame_idx % self.redetection_interval == 0) or (len(self.tracks) < self.minimum_num_tracks):
            # Ignore black/white
            mask = cv2.inRange(frame_grayscale, 1, 254)

            # Ignore existing tracking points
            for x, y in [np.int32((tr[-1][0], tr[-1][1])) for tr in self.tracks]:
                cv2.circle(mask, (x, y), self.point_mask_radius, 0, -1)

            p = cv2.goodFeaturesToTrack(frame_grayscale, mask=mask, **self.feature_params)

            if p is not None:
                for x, y in np.float32(p).reshape(-1, 2):
                    self.tracks.append([(x, y, R, agl, now, self.frame_idx)])

        self.previous_frame = frame_grayscale

        return display_frame
