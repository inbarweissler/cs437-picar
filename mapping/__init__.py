from __future__ import annotations

import dataclasses
import os.path
import time
from datetime import datetime
from typing import List, Tuple, Optional

import cv2
import numpy as np
import matplotlib.pyplot as plt
import pyastar2d
from pyastar2d import Heuristic

import picar_4wd as fc

# types
AngleDeg = int
DistanceCm = float

# const
GRID_SIZE_CM = 140
GRID_RESOLUTION_CM = 4  # good for A* maneuvers

MAX_DISTANCE_TH = 100
MAX_DISTANCE_STD_TH = 1

MAPPING_MEMORY_FACTOR = 0.5


class ScanningGrid:
    def __init__(
            self,
            max_dist_cm: int = GRID_SIZE_CM,
            grid_res_cm: int = GRID_RESOLUTION_CM
    ):
        self._len_x = int(max_dist_cm / grid_res_cm)
        self._len_y = int(max_dist_cm / grid_res_cm)
        self._x0_cm = -max_dist_cm // 2
        self._y0_cm = 0
        self._max_dist_cm = max_dist_cm
        self._grid_res_cm = grid_res_cm

    @property
    def res_cm(self):
        return self._grid_res_cm

    @property
    def max_dist_cm(self):
        return self._max_dist_cm

    @property
    def len_x(self):
        return self._len_x

    @property
    def len_y(self):
        return self._len_y

    @property
    def x_origin_cm(self):
        return self._x0_cm

    @property
    def y_origin_cm(self):
        return self._y0_cm


@dataclasses.dataclass
class MapCoordinate:
    x: DistanceCm
    y: DistanceCm

    def get_heading_angle(self, to: MapCoordinate) -> AngleDeg:
        return int(np.degrees(np.arctan2((to.x - self.x), (to.y - self.y))))


@dataclasses.dataclass(frozen=False)
class Maneuver:
    heading: AngleDeg
    forward: DistanceCm
    end_position: MapCoordinate


class Mapper:
    def __init__(self,
                 grid: ScanningGrid = ScanningGrid(),
                 scan_res: AngleDeg = 10,
                 scan_span: AngleDeg = 90,
                 num_scans: int = 5,  # to reduce noise impact
                 output_path: Optional[str] = None
                 ):
        self._grid = grid
        self._current_map = np.zeros((self._grid.len_x, self._grid.len_y), dtype='float32')
        self._car_position = MapCoordinate(0, 0)
        self._car_heading = 0
        self._map_history: List[np.ndarray] = []
        self._scan_res = scan_res
        self._scan_span = scan_span
        self._num_scans = num_scans
        self._output_path = "./" if output_path is None else output_path

    def update_grid(self,
                    car_position: MapCoordinate,
                    car_heading: AngleDeg,
                    ):
        self._car_position = car_position
        self._car_heading = car_heading

        data: List[Tuple[DistanceCm, AngleDeg]] = []
        for angle_deg in range(-self._scan_span, self._scan_span, self._scan_res):
            fc.servo.set_angle(-angle_deg)
            time.sleep(0.001)
            dist_list = [fc.us.get_distance() for _ in range(self._num_scans)]
            dist_list = [dist for dist in dist_list if dist is not None]
            if len(dist_list):
                dist_cm = float(np.mean(dist_list))
                dist_std = float(np.std(dist_list))
                if dist_cm <= MAX_DISTANCE_TH and dist_std < MAX_DISTANCE_STD_TH:
                    data.append((dist_cm, angle_deg + car_heading))  # angle is represented opposite to x_axis

        self._current_map *= 0
        for obj_dist, obj_angle in data:
            obj_position = MapCoordinate(
                x=int(self._car_position.x + obj_dist * np.sin(np.radians(obj_angle))),
                y=int(self._car_position.y + obj_dist * np.cos(np.radians(obj_angle)))
            )
            grid_pos = self.__coord2grid(obj_position)
            if grid_pos:
                self._current_map[grid_pos] += 1
        obstacles = cv2.dilate(self._current_map, kernel=np.ones((2, 2), np.uint8), iterations=1)
        self._current_map = 50 * cv2.blur(obstacles, (5, 5)) + 100 * obstacles  # we use blur for obstacles clearing
        self._current_map += 1
        if len(self._map_history) > 0:
            self._current_map += MAPPING_MEMORY_FACTOR * self._map_history[-1]
        self._map_history.append(self._current_map.copy())

    @property
    def current_map(self):
        return self._current_map

    def plot_map(self):
        plt.pcolor(
            np.linspace(self._grid.x_origin_cm, self._grid.max_dist_cm // 2, self._grid.len_x),
            np.linspace(self._grid.y_origin_cm, self._grid.max_dist_cm, self._grid.len_y),
            self._current_map.T,
        )
        plt.gca().set_aspect('equal', 'box')
        plt.xlabel('x [cm]')
        plt.ylabel('y [cm]')
        plt.title(f'Mapping')
        # plt.show()
        file_name = os.path.join(self._output_path, f'map_{datetime.now().strftime("%H:%M:%S")}.jpg')
        plt.savefig(file_name)
        plt.clf()

    def plot_navigation_plan(self, destination: MapCoordinate) -> List[Maneuver]:
        origin_grid = self.__coord2grid(self._car_position)
        destination_grid = self.__coord2grid(destination)

        weights = self._current_map
        # The start and goal coordinates are in matrix coordinates (i, j).
        abs_heading = abs(self._car_position.get_heading_angle(destination))
        heuristic = Heuristic.ORTHOGONAL_Y if 45 < abs_heading < 135 else Heuristic.ORTHOGONAL_X
        path = pyastar2d.astar_path(
            weights=weights,
            start=origin_grid,
            goal=destination_grid,
            allow_diagonal=True,
            heuristic_override=heuristic
        )
        path_coord = [MapCoordinate(*self.__grid2coord(x_ind, y_ind)) for x_ind, y_ind in path]
        maneuvers = self.__get_maneuvers(path_coord)
        path_plot = np.array([(coord.x, coord.y) for coord in path_coord])

        plt.pcolor(
            np.linspace(self._grid.x_origin_cm, self._grid.max_dist_cm // 2, self._grid.len_x),
            np.linspace(self._grid.y_origin_cm, self._grid.max_dist_cm, self._grid.len_y),
            self._current_map.T,
        )
        plt.plot(path_plot[:, 0], path_plot[:, 1], '-*r')
        plt.plot(self._car_position.x, self._car_position.y, 'og')
        plt.plot(destination.x, destination.y, 'xg')
        plt.gca().set_aspect('equal', 'box')
        plt.xlabel('x [cm]')
        plt.ylabel('y [cm]')
        plt.title(f'Navigation plan, step {len(self._map_history)}')
        # plt.show()
        file_name = os.path.join(self._output_path, f'navigation_{datetime.now().strftime("%H:%M:%S")}.jpg')
        plt.savefig(file_name)
        plt.clf()
        return maneuvers

    def __coord2grid(self, position: MapCoordinate) -> Optional[Tuple[int, int]]:
        x_ind = int((position.x - self._grid.x_origin_cm) / self._grid.res_cm)
        y_ind = int((position.y - self._grid.y_origin_cm) / self._grid.res_cm)
        if 0 <= x_ind < self._grid.len_x and 0 <= y_ind < self._grid.len_y:
            return x_ind, y_ind
        else:
            return None

    def __grid2coord(self, x_ind: int, y_ind: int) -> Tuple[float, float]:
        return x_ind * self._grid.res_cm + self._grid.x_origin_cm, \
               y_ind * self._grid.res_cm + self._grid.y_origin_cm

    @staticmethod
    def __get_maneuvers(path_coord: List[MapCoordinate]) -> List[Maneuver]:
        result: List[Maneuver] = []
        for i in range(1, len(path_coord)):
            heading_ang = path_coord[i - 1].get_heading_angle(path_coord[i])
            fw = np.sqrt(
                np.power(path_coord[i].x - path_coord[i - 1].x, 2) + np.power(path_coord[i].y - path_coord[i - 1].y, 2))
            result.append(Maneuver(heading_ang, fw, path_coord[i]))
        return result
