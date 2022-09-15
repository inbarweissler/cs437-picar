import sys
sys.path.append('/home/pi/cs437-picar')

from mapping import Mapper, MapCoordinate


if __name__ == '__main__':
    mapper = Mapper()
    car_position = MapCoordinate(0, 0)
    car_heading = 0
    mapper.update_grid(car_position, car_heading)
    mapper.plot_map()
