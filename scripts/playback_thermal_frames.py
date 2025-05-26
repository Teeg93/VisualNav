import random

import time
import cv2
import sys
import os
import json
import numpy as np
from threading import Thread
import argparse
import datetime

import tkinter as tk
from tkinter import ttk
root = tk.Tk()
root.title('Playback')

min_temp = tk.IntVar()
max_temp = tk.IntVar()
frame_index = tk.IntVar()
image_scale = tk.IntVar()
automatic_bracketing = tk.IntVar()
optical_flow = tk.IntVar()


play_video = False

sys.path.append('../src')
from MetadataHandler import MetadataHandlerCSV
from ThermalTools import raw_to_thermal_frame, ThermalFlow, compute_thermal_bracket, compute_thermal_bracket_8_bit, scale_frame
from OpticalFlow import OpticalFlow


# Parse command line arguments
parser = argparse.ArgumentParser(description="Playback from an image  directory")

parser.add_argument('-d', '--directory', type=str, required=True, help="Directory which contains the video file")
parser.add_argument('-tmin', type=float, required=True, help="Minimum temperature (8 bit)")
parser.add_argument('-tmax', type=float, required=True, help="Maximum temperature (8 bit)")

args = parser.parse_args()


thermal_directory = args.directory
files = os.listdir(thermal_directory)
image_files = [x for x in files if x.endswith('.png') or x.endswith('.jpg') or x.endswith('.jpeg')]
if image_files is None or len(image_files) == 0:
    print(f"Could not find any files in directory <{thermal_directory}>")
    sys.exit(1)

image_files = sorted(image_files)

metadata_file = os.path.join(thermal_directory, "mavlink.csv")
if not os.path.exists(metadata_file):
    print(f"No metadata available - could not locate <mavlink.csv> in <{thermal_directory}>")
    sys.exit(1)

metadata_handler = MetadataHandlerCSV(metadata_file)




def create_text_field(root, label, current_value, row):
    text_label = ttk.Label(
        root,
        text=label
    )
    text_label.grid(
        column=0,
        row=row,
        sticky='w'
    )

    variable = tk.Text(
            root,
            width=35,
            height=8,
        )

    variable.grid(
        column=1,
        row=row,
        sticky='we',
    )
    return variable

def create_gui_item(root, label, current_value, variable, row, editable=True):
    entry_label = ttk.Label(
        root,
        text=label
    )
    entry_label.grid(
        column=0,
        row=row,
        sticky='w'
    )

    if editable:
        entry = ttk.Entry(
            root,
            textvariable=variable,
            validate='all',
        )
    else:
        entry = ttk.Entry(
            root,
            textvariable=variable,
            validate='all',
            state='disabled',
        )

    entry.delete(0, tk.END)
    entry.insert(0, current_value)
    entry.grid(
        column=1,
        row=row,
        sticky='we',
    )
    variable.set(current_value)

def create_gui_button(root, label, callback, row):
    b = tk.Button(root, text=label, command=callback)
    b.grid(column=0,
           row=row,
           sticky='we',
    )

def create_slider(root, label, min_val, max_val, current_value, variable, row):
    slider_label = ttk.Label(
        root,
        text=label,
    )
    slider_label.grid(
        column=0,
        row=row,
        sticky='w'
    )
    slider = ttk.Scale(
        root,
        from_=min_val,
        to=max_val,
        orient='horizontal',
        variable=variable,
        length=200,
    )
    slider.grid(
        column=1,
        row=row,
        sticky='we'
    )

    text_entry = tk.Entry(
        width=10,
        textvariable=variable,
        validate='all'
    )
    text_entry.grid(
        column=2,
        row=row,
        sticky='we'
    )

    variable.set(current_value)


def create_checkbox(root, label, current_value, variable, row):
    checkbox = ttk.Checkbutton(root, text=label, variable=variable, onvalue=1, offvalue=0)
    checkbox.grid(
        column = 0,
        row=row,
        sticky='w')
    variable.set(current_value)


def pause_callback():
    global play_video
    print("Paused")
    play_video = False

def play_callback():
    global play_video
    print("Resumed")
    play_video = True


def draw_metadata(frame, metadata):
    row_idx = 0

    x_offset = 1
    y_offset = 30
    scalar = 30

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1

    if len(frame.shape) > 2 and frame.shape[2] >= 2:
        color = (255, 255, 255)
    else:
        color = 255
    thickness = 2
    org = (x_offset, row_idx*10 + y_offset)

    filename = metadata.filename
    timestamp = float(metadata.filename.split(".")[0])/1000
    time_string = datetime.datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S')

    frame = cv2.putText(frame, f"{time_string}", org, font, font_scale, color, thickness, cv2.LINE_AA)
    row_idx += 1

    accepted_keys = ["frame", "lat", "lon", "alt", "airspeed", "groundspeed", "groundcourse", "agl"]
    dat = metadata.as_dict()
    for key, val in dat.items():
        if val is not None and val and (key == "lat" or key == "lon"):
            val = f"{val:.5f}"
        if val is not None and val and (key == "groundspeed" or key == "airspeed" or key == "groundcourse"):
            val = f"{val:.1f}"
        if val is not None and val and (key == "alt" or key == "agl"):
            val = f"{val:.0f}"
        


        if not key in accepted_keys:
            continue
        org = (x_offset, row_idx*scalar + y_offset)
        frame = cv2.putText(frame, f"{key}: {val}", org, font, font_scale, color, thickness, cv2.LINE_AA)
        row_idx += 1


    return frame



def video_loop():
    global play_video
    global frame_index


    previous_timestamp = None
    previous_iteration_time = time.time()

    previous_frame_index = 0

    eof_flag = False

    tf = OpticalFlow(point_mask_radius=20, maximum_track_len=10)

    while True:

        thermal_window_name = "Thermal"
        #window = cv2.namedWindow(thermal_window_name, cv2.WINDOW_NORMAL)


        current_frame_index = frame_index.get()
        if current_frame_index >= (len(image_files)-1) and not eof_flag:
            print("Reached end of video")
            eof_flag = True

        if current_frame_index >= (len(image_files) -1):
            current_frame_index = len(image_files) - 1
            frame_index.set(len(image_files)-1)

        if current_frame_index < len(image_files) -1:
            eof_flag = False

        if not (current_frame_index - previous_frame_index == 1):
            previous_timestamp = None
        previous_frame_index = current_frame_index

        image_filename = image_files[current_frame_index]
        metadata = metadata_handler.get_metadata_from_filename(image_filename)
        timestamp = int(image_filename.split(".")[0]) / 1000


        if previous_timestamp is None:
            dt = 0
        else:
            dt = timestamp - previous_timestamp

        previous_timestamp = timestamp

        full_filepath = os.path.join(thermal_directory, image_filename)
        frame_16bit = cv2.imread(full_filepath, -1)
        frame_16bit = frame_16bit[:233,:]


        if automatic_bracketing.get():
            if args.tmax and args.tmin:
                mn, mx = compute_thermal_bracket_8_bit(frame_16bit, args.tmin, args.tmax)
                min_temp.set(mn)
                max_temp.set(mx)
            else:
                mn, mx = compute_thermal_bracket(frame_16bit)
                min_temp.set(mn)
                max_temp.set(mx)

        _min_temp = min_temp.get()
        _max_temp = max_temp.get()


        if args.tmin and args.tmax:
            frame_8bit = scale_frame(frame_16bit, args.tmin, args.tmax, _min_temp, _max_temp)
        else:
            frame_8bit = raw_to_thermal_frame(frame_16bit, _min_temp, _max_temp)

        if optical_flow.get():
            frame_8bit = tf.pass_frame(frame_8bit, None, None)


        _scale = image_scale.get()
        frame_8bit = cv2.resize(frame_8bit, None, fx=_scale, fy=_scale)
        frame_8bit = draw_metadata(frame_8bit, metadata)

        while time.time() - previous_iteration_time < dt:
            time.sleep(0.001)
        previous_iteration_time = time.time()
        cv2.imshow(thermal_window_name, frame_8bit)

        ret = cv2.waitKey(1)

        if ret == 81: #Left
            if frame_index.get() <= 0:
                continue
            else:
                frame_index.set(frame_index.get() - 1)
                continue

        elif ret == 83: #Right
            if frame_index.get() >= len(image_files):
                continue
            else:
                frame_index.set(frame_index.get() + 1)
                continue
        
        elif ret == 85: #Page Up
            if frame_index.get() >= len(image_files):
                continue
            else:
                frame_index.set(np.clip(frame_index.get() + 10, 0, len(image_files)-1))
                continue

        elif ret == 86: #Page Down
            if frame_index.get() <= 0:
                continue
            else:
                frame_index.set(np.clip(frame_index.get() - 10, 0, len(image_files)-1))
                continue
        

        
        elif ret == 32: #Spacebar
            play_video =  not play_video

        if not eof_flag and play_video:
            frame_index.set(frame_index.get() + 1)


def main():
    row_idx = 0

    create_gui_button(root, "Play", play_callback,  row_idx); row_idx += 1
    create_gui_button(root, "Pause", pause_callback,    row_idx); row_idx += 1

    create_slider(root, "Min Temp", -30, 70, 0, min_temp, row_idx); row_idx += 1
    create_slider(root, "Max Temp", -30, 70, 50, max_temp, row_idx); row_idx += 1

    create_slider(root, "Frame Index", 0, len(image_files)-1, 0, frame_index, row_idx); row_idx += 1
    create_slider(root, "Scale", 1, 10, 4, image_scale, row_idx); row_idx += 1

    create_checkbox(root, "Auto Bracketing", 0, automatic_bracketing, row_idx); row_idx += 1
    create_checkbox(root, "Optical Flow", 0, optical_flow, row_idx); row_idx += 1


    t = Thread(target=video_loop)
    t.daemon=True
    t.start()

    root.mainloop()

    print("Exiting...")

if __name__ == "__main__":
    main()