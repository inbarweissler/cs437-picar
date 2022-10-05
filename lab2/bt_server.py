import sys
sys.path.append('/home/pi/cs437-picar')
import time
from typing import Dict, Callable
import picar_4wd as fc
from picar_4wd import utils
import bluetooth

hostMACAddress = "E4:5F:01:B4:29:1B"  # The address of Raspberry PI Bluetooth adapter on the server.
# The server might have multiple Bluetooth adapters.
port = 0
backlog = 1
size = 1024


car_command: Dict[str, Callable[[int], None]] = {
    "f": fc.forward,
    "b": fc.backward,
    "r": fc.turn_right,
    "l": fc.turn_left,
}


s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.bind((hostMACAddress, port))
s.listen(backlog)
print("listening on port ", port)
client = None
fc.start_speed_thread()
t1 = time.time()
distance = 0.0
try:
    client, clientInfo = s.accept()
    while 1:
        data = client.recv(size)  # receive 1024 Bytes of message in binary format
        received_data = data.decode("utf-8")
        car_direction = 'h'
        if received_data != "":
            direction = received_data[0]
            # speed_level = received_data[1:]
            if direction in ('f', 'b', 'r', 'l'):  # and speed_level.isdecimal():
                car_direction = direction
                speed_int = 2  # int(speed_level)
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
        if car_direction in ('f', 'b'):
            distance += speed * dt
        cpu_temp = round(fc.cpu_temperature(), 1)
        battery = round(utils.power_read(), 1)
        telemetries = f'["{direction}","{speed:0>5.2f}","{round(distance, 2):0>6.2f}","{cpu_temp:0>5.2f}","{battery:0>5.2f}","{received_data}"]'
        client.send(bytes(telemetries, "utf-8"))  # Echo back to client
except Exception as ex:
    print(ex)
    print("Closing socket")
if client is not None:
    client.close()
s.close()
