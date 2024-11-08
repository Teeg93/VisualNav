import pandas
import json

class MavlinkMetadata:
    def __init__(self, filename, frame, timestamp, lat, lon, alt, airspeed, groundspeed, throttle, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed, exposure_start, exposure_end, attitude_extended, groundcourse=None):
        self.filename = filename
        self.frame = frame
        self.timestamp = timestamp
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.airspeed = airspeed
        self.groundspeed = groundspeed
        self.throttle = throttle
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.rollspeed = rollspeed
        self.pitchspeed = pitchspeed
        self.yawspeed = yawspeed
        self.exposure_start = exposure_start
        self.exposure_end = exposure_end
        self.attitude_extended = attitude_extended
        self.groundcourse = groundcourse

    def __str__(self):
        string = ""
        string += f"Filename: {self.filename}\n"
        string += f"Frame: {self.frame}\n"
        string += f"Timestamp: {self.timestamp}\n"
        string += f"Exposure start: {self.exposure_start}\n"
        string += f"Exposure end: {self.exposure_end}\n"
        string += f"Latitude: {self.lat}\n"
        string += f"Longitude: {self.lon}\n"
        string += f"Altitude: {self.alt}\n"
        string += f"Airspeed: {self.airspeed}\n"
        string += f"Groundspeed: {self.groundspeed}\n"
        string += f"Throttle: {self.throttle}\n"
        string += f"Roll: {self.roll}\n"
        string += f"Pitch: {self.pitch}\n"
        string += f"Yaw: {self.yaw}\n"
        string += f"Rollspeed: {self.rollspeed}\n"
        string += f"Pitchspeed: {self.pitchspeed}\n"
        string += f"Yawspeed: {self.yawspeed}\n"
        string += f"Attitude Extended: {json.dumps(self.attitude_extended, indent=4)}\n"
        string += f"Groundcourse: {json.dumps(self.groundcourse, indent=4)}\n"
        return string

class MetadataHandlerCSV:
    def __init__(self, f):
        self.df = pandas.read_csv(f, delimiter="%")


    def get_metadata_from_filename(self, filename):
        row = self.df.loc[self.df['filename'] == filename]
        if row.empty:
            return None
        return self.pack_metadata_from_row(row)

    def get_metadata_from_frame(self, frame):
        row = self.df.loc[self.df['frame'] == frame]
        if row.empty:
            return None
        return self.pack_metadata_from_row(row)

    def get_final_frame_index(self):
        frames_list = self.df['frame'].tolist()
        sorted_frames_list = sorted(frames_list)
        return sorted_frames_list[-1]

    def get_first_frame_index(self):
        frames_list = self.df['frame'].tolist()
        sorted_frames_list = sorted(frames_list)
        return sorted_frames_list[0]

    def pack_metadata_from_row(self, row):
        filename = row.values[0][0]
        frame = row.values[0][1]
        timestamp = row.values[0][2]/1000000
        if timestamp == 0:
            timestamp = int(filename.split(".")[0])/1000000000
        lat = row.values[0][3]
        lng = row.values[0][4]
        alt = row.values[0][5]
        airspeed = row.values[0][6]
        groundspeed = row.values[0][7]
        throttle = row.values[0][8]
        roll = row.values[0][9]
        pitch = row.values[0][10]
        yaw = row.values[0][11]
        rollspeed = row.values[0][12]
        pitchspeed = row.values[0][13]
        yawspeed = row.values[0][14]
        exposure_start = row.values[0][15]
        exposure_end = row.values[0][16]
        try:
            attitude_extended = json.loads(row.values[0][17])
        except (json.decoder.JSONDecodeError, TypeError):
            attitude_extended = None

        try:
            groundcourse = row.values[0][18]
        except:
            groundcourse = None

        metadata = MavlinkMetadata(filename,
                                   frame,
                                   timestamp,
                                   lat,
                                   lng,
                                   alt,
                                   airspeed,
                                   groundspeed,
                                   throttle,
                                   roll,
                                   pitch,
                                   yaw,
                                   rollspeed,
                                   pitchspeed,
                                   yawspeed,
                                   exposure_start,
                                   exposure_end,
                                   attitude_extended,
                                   groundcourse
                                   )

        return metadata
