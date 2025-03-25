import time
import argparse
from VideoStream import VideoStreamClient

parser = argparse.ArgumentParser(description="Video client")

parser.add_argument('-i', '--ip-address', type=str, default='127.0.0.1', help="IP address of the server")
parser.add_argument('-p', '--port', type=int, default=8888, help="Port number of the server")

args = parser.parse_args()



client = VideoStreamClient(args.ip_address, args.port, daemon=False)

try:
    client.run()
except KeyboardInterrupt:
    print("Exiting...")
    client.stop()