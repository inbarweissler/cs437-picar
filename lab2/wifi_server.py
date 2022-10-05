import sys

sys.path.append('/home/pi/cs437-picar')
import time
from typing import Dict, Callable
import picar_4wd as fc
from picar_4wd import utils
import socket

HOST = "192.168.8.24"  # IP address of your Raspberry PI
PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

car_command: Dict[str, Callable[[int], None]] = {
    "f": fc.forward,
    "b": fc.backward,
    "r": fc.turn_right,
    "l": fc.turn_left,
}

car_direction_dict: Dict[str, str] = {
    "f": "forward",
    "b": "backward",
    "r": "right",
    "l": "left",
}

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen()
    client = None
    fc.start_speed_thread()
    t1 = time.time()
    distance = 0.0
    try:
        while 1:
            client, clientInfo = s.accept()
            data = client.recv(1024)  # receive 1024 Bytes of message in binary format
            received_data = data.strip().decode("utf-8")
            car_direction = 'hold'
            if received_data != "":
                direction = received_data[0]
                speed_level = received_data[1:]
                if direction in ('f', 'b', 'r', 'l') and speed_level.isdecimal():
                    car_direction = car_direction_dict[direction]
                    speed_int = int(speed_level)
                    car_command[direction](speed_int)
                else:
                    fc.stop()
                print(f'server recv from: {clientInfo}, message: {received_data}')
            else:
                fc.stop()
            t2 = time.time()
            dt = t2 - t1
            t1 = t2
            speed = round(fc.speed_val(), 2)
            if car_direction in ('forward', 'backward'):
                distance += speed * dt
            cpu_temp = round(fc.cpu_temperature(), 1)
            battery = round(utils.power_read(), 1)
            telemetries = f'["{car_direction}","{speed}","{round(distance, 2)}","{cpu_temp}","{battery}","{received_data}"]'
            client.sendall(bytes(telemetries, "utf-8"))  # Echo back to client
    except Exception as ex:
        print(ex)
        print("Closing socket")
        if client is not None:
            client.close()
        s.close()
