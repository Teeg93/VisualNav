import random

import time
import cv2
import sys
import json
from threading import Thread

import tkinter as tk
from tkinter import ttk
root = tk.Tk()
root.title('Playback')

min_temp = tk.IntVar()
max_temp = tk.IntVar()

play_video = False


def downsample(frame, min_value, max_value):
    range = max_value - min_value
    frame = cv2.convertScaleAbs(frame, alpha=255 / range, beta=(-255 * min_value) / range)
    return frame

def rawToThermalFrame(rawFrame, min, max):
    """
    Use the device configuration to scale the 16-bit image to an 8 bit image, given the min & max temperatures.

    :param rawFrame: 16 bit radiometric image
    :return: 8 bit temperature scaled image
    """
    # Get temperature range limits
    _dt = max - min

    # Equivalent to 255*( ( ( (x-20000) / 100) -_min ) / ( _max - _min) )
    # Much faster than floating point temperature operations
    temp = cv2.convertScaleAbs(rawFrame, alpha=255 / (100 * _dt), beta=-255 * (273 / _dt + min / _dt))
    return temp


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




def pause_callback():
    global play_video
    print("Paused")
    play_video = False

def play_callback():
    global play_video
    print("Resumed")
    play_video = True


def video_loop():
    while True:
        time.sleep(1)
        _min_temp = min_temp.get()
        _max_temp = max_temp.get()

        if play_video:
            print(_min_temp)
            print(_max_temp)


def main():
    row_idx = 0

    create_gui_button(root, "Play", play_callback,  row_idx); row_idx += 1
    create_gui_button(root, "Pause", pause_callback,    row_idx); row_idx += 1

    #create_gui_item(root, "Min Temp", 0, min_temp, row_idx); row_idx += 1
    #create_gui_item(root, "Max Temp", 50, max_temp, row_idx); row_idx += 1
    create_slider(root, "Min Temp", -30, 70, 0, min_temp, row_idx); row_idx += 1
    create_slider(root, "Max Temp", -30, 70, 50, max_temp, row_idx); row_idx += 1


    t = Thread(target=video_loop)
    t.daemon=True
    t.start()

    root.mainloop()

    print("Exiting...")

if __name__ == "__main__":
    main()