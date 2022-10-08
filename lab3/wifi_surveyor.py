import os
import sys
from typing import List, Dict, Set

sys.path.append('/home/pi/cs437-picar')
import picar_4wd as fc
import subprocess
import csv
from time import sleep

Header = str
CellData = Dict[str, str]


def get_iw_scan(device_name: str) -> Dict[str, CellData]:
    cmd = f"sudo iwlist {device_name} scan"
    lines = []
    while len(lines) < 1 or "Scan completed" not in lines[0]:
        sleep(0.5)
        output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
        lines = [line.decode("utf-8").strip() for line in output.stdout.readlines()]
    ind = 1
    cells: Dict[str, CellData] = {}
    while ind < len(lines):
        if lines[ind].startswith("Cell"):
            data_dict = {}
            cell_title = lines[ind].split(" - ", 1)
            ind += 1
            while not lines[ind].startswith("Cell") and ind < len(lines) - 1:
                line = lines[ind].split(":", 1)
                if len(line) == 1:
                    line = line[0]
                    if '=' in line:
                        for split_line in line.split(" ", 1):
                            sub_line = split_line.split('=', 1)
                            data_dict[sub_line[0]] = sub_line[1]
                    elif 'Mb/s' in line and 'Bit Rates' in data_dict:
                        data_dict['Bit Rates'] += " " + line
                    else:
                        print(line)
                else:
                    if not line[0].startswith("Extra") and not line[1].__contains__("Unknown"):
                        if line[0] in data_dict:
                            data_dict[line[0]] += " " + line[1]
                        else:
                            data_dict[line[0]] = line[1].strip()
                ind += 1
            mac_address = cell_title[1].split(": ", 1)
            data_dict[mac_address[0]] = mac_address[1]
            cells[cell_title[0]] = data_dict
        else:
            ind += 1
    return cells


if __name__ == "__main__":
    n_measurements = 8
    wait_time = 20
    device_name = 'wlan0'
    file_data: List[CellData] = []
    cell_info: Set[Header] = set()
    for call_id in range(n_measurements):
        for tx_power in [31, 24, 18]:
            fc.servo.set_angle_and_wait(-90)
            os.system(f"sudo iwconfig {device_name} txpower {tx_power}")
            cells = get_iw_scan(device_name)
            for _, cell_data in cells.items():
                cell_data["call_id"] = str(call_id)
                cell_data["tx_power"] = str(tx_power)
                file_data.append(cell_data)
                cell_info = cell_info.union(set(cell_data.keys()))
        os.system(f"sudo iwconfig {device_name} txpower 31")
        fc.servo.set_angle_and_wait(0)
        sleep(wait_time)

    with open('test.csv', 'w') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=cell_info)
        writer.writeheader()
        writer.writerows(file_data)
    fc.servo.set_angle_and_wait(90)
    print("Done!")
