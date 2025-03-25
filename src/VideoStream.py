import cv2
import socket
import time
import pickle
import struct
from threading import Thread
from FlightUtils.ImageFragmentation import *
import numpy as np


class VideoStreamServer:
    # Create a socket server
    def __init__(self, ip_address="0.0.0.0", port=8888, compression_quality=10):
        self.ip_address = ip_address
        self.port = port
        self.compression_quality = compression_quality

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((ip_address, port))
        self.socket.settimeout(0.2)

        self.frame_id = 0

        self.t = Thread(target=self.run)
        self.t.daemon = True
        self.t.start()
        self.enc_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.compression_quality]

        self.active_clients = {}


    def run(self):
        print(f"Listening on {self.ip_address}:{self.port}")

        while True:
            try:
                # Receive messages from the client. Update the active clients dict
                message, address = self.socket.recvfrom(1024)
                key = str(address[0]) + ":" + str(address[1])
                self.active_clients.update({key:time.time()})

            except TimeoutError:
                pass
        
            # Check if any clients are inactive
            removal_keys = []
            for key, last_msg_time in self.active_clients.items():
                if time.time() - last_msg_time > 10:
                    removal_keys.append(key)
            
            # Remove inactive clients
            for key in removal_keys:
                del(self.active_clients[key])



    
    def send_frame(self, frame):

        # If there are no active clients, don't waste time encoding the image
        if len(self.active_clients) == 0:
            return

        if frame is not None:
            # Serialize the frame to bytes
            res, enc_img = cv2.imencode('.jpg', frame, self.enc_param)
            if not res:
                print("Unable to encode image. Not sending")
                return
            
            # Fragment the image to send via UDP
            fragmenter = ImageFragmentation(self.frame_id, enc_img.tostring())
            if not fragmenter.is_valid():
                print("Data is too long to fragment. Consider compressing images")
                return
            data_fragments = fragmenter.fragment()

            # Send the data to each active client
            for address in self.active_clients:
                ip, port = address.split(":")
                port = int(port)
                add = (ip, port)
                for df in data_fragments:
                    self.socket.sendto(df, add)

            self.frame_id += 1
            self.frame_id = self.frame_id % 255
        else:
            print("Frame is None you donkey")
        

class VideoStreamClient:
    # Create a socket client
    def __init__(self, ip_address, port, daemon=False):
        self.ip_address = ip_address
        self.port = port
        self.addr = (ip_address, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(2.0)

        self.defrag = ImageDefragmentation()

        self._stop = False

        if daemon:
            self.t = Thread(target=self.run)
            self.t.daemon = True
            self.t.start()

    def stop(self):
        self._stop = True
         
    def run(self):
        send_data = b""

        while True:
            if self._stop:
                break

            try:
                # Send a keep-alive signal
                self.socket.sendto(send_data, self.addr) 

                # Receive data from the streamer
                received_data = self.socket.recv(65535)

                # Defragment the data
                ret = self.defrag.pass_fragment(received_data)

                # If the defragmenter detects a complete image, process it
                if ret:
                    nparr = np.fromstring(ret, np.uint8)
                    if nparr.any():
                        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                        # Display the received frame
                        cv2.imshow('Client Video', frame)

                # Press ‘q’ to quit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            except TimeoutError as e:
                time.sleep(0.01)
                continue

        # Release resources
        cv2.destroyAllWindows()
        video_client_socket.close()